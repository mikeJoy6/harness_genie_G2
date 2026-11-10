"""真机器人:复用 agi@g2_robot 的 ArmMotion(规划、执行)和头部 RGBD 工具。

运行前要 source ~/agi@g2_robot/env_g2.sh,并且 hardware.launch.py 已经在跑。
"""
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster

from genie_robot_agibot_g2_motion.arm_motion import CART_SPEED, ArmMotion
from task_board_pose.common.sensor_node import SensorNode
from task_board_pose.vision.calib import CALIB
from task_board_pose.vision.rgbd_source import RosSource
from task_board_pose.vision.scene3d import Rgbd

from .color_object import find_objects, merge_frames

ARM = "left"


def slow_down(plan, speed):
    """把轨迹按比例放慢到末端 speed m/s,形状不变。"""
    k = CART_SPEED / speed
    if k <= 1.0:
        return plan
    for point in plan.traj.points:
        t = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) * k
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(t * 1e9), 10**9)
        point.velocities = [v / k for v in point.velocities]
        if point.accelerations:
            point.accelerations = [a / k / k for a in point.accelerations]
    plan.duration *= k
    return plan


class RosBackend:
    def __init__(self, vision_cfg, speed_m_s, debug_dir="traces"):
        self.vision = vision_cfg
        self.speed = speed_m_s
        self.debug_dir = Path(debug_dir)
        self.rgbd = Rgbd()
        self.block_xyz = None

        rclpy.init()
        self.node = rclpy.create_node("robot_harness")
        self.arm = ArmMotion("robot_harness_arm")
        self.camera = SensorNode("robot_harness_camera")
        self.tf = TransformBroadcaster(self.node)
        self.node.create_timer(0.1, self._publish_tf)

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.arm)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

        try:
            self._wait_until_ready()
        except Exception:
            self.close()
            raise

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:      # 退出时 executor 关闭的竞争,不影响结果
            pass

    def _wait_until_ready(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not self.arm.ready():
            time.sleep(0.05)
        if not self.arm.ready():
            raise RuntimeError("10s 没收到 /robot_description 或 /joint_states,hardware.launch.py 在跑吗?")
        if not self.arm._sv.wait_for_service(timeout_sec=5.0):   # 否则 ArmMotion 会跳过自碰撞检查
            raise RuntimeError("/check_state_validity 不可用,move_group 在跑吗?")

    def locate_object(self, color, ranges):
        source = RosSource(sensor=self.camera)
        per_frame, rejected, last = [], [], None
        deadline = time.monotonic() + self.vision["timeout_s"]
        try:
            while len(per_frame) < self.vision["frames"] and time.monotonic() < deadline:
                frame = source.next(None)
                if frame is None:
                    time.sleep(0.01)
                    continue
                bgr, depth_mm, T_base_head, _ = frame
                depth_m = depth_mm.astype(np.float32) / 1000.0
                points, _ = self.rgbd.points_base(depth_m, T_base_head)
                found, why = find_objects(bgr, depth_m, points, CALIB["depth"]["fx"], ranges, self.vision)
                per_frame.append(found)
                rejected = why or rejected
                last = bgr
        finally:
            source.close()

        if not per_frame:
            return {"ok": False, "error": "没有收到头部相机画面(/gdk/camera/head_color、head_depth、/gdk/tf)"}
        block, error = merge_frames(per_frame, self.vision["min_good_frames"], self.vision["max_spread_m"],
                                    self.left_end_xyz())
        self._save_debug_image(last, per_frame[-1], block)
        if block is None:
            return {"ok": False, "error": error, "rejected": rejected[:3]}

        self.block_xyz = block["xyz_m"]
        return {"ok": True, "frame": "base_link", "tf": "target_object", "color": color,
                "xyz_m": [round(float(v), 4) for v in block["xyz_m"]],
                "pixels": block["pixels"], "depth_m": block["depth_m"], "size_m": block["size_m"],
                "candidates": block["candidates"], "selected": "离左臂末端最近",
                "distance_to_left_end_m": block["distance_to_left_end_m"]}

    def _save_debug_image(self, bgr, candidates, block):
        """黄框是所有候选,绿框是最终选中的目标。"""
        image = bgr.copy()
        for c in candidates:
            cv2.rectangle(image, (c["x"], c["y"]), (c["x"] + c["w"], c["y"] + c["h"]), (0, 255, 255), 1)
        if block is not None:
            x, y, w, h = block["x"], block["y"], block["w"], block["h"]
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        self.debug_dir.mkdir(exist_ok=True)
        cv2.imwrite(str(self.debug_dir / "last_object.jpg"), image)

    def _publish_tf(self):
        if self.block_xyz is None:
            return
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "target_object"
        t.transform.translation.x = float(self.block_xyz[0])
        t.transform.translation.y = float(self.block_xyz[1])
        t.transform.translation.z = float(self.block_xyz[2])
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)

    def left_end_xyz(self):
        """arm_l_end_link 在 base_link 下的位置:由实测关节角正运动学得到,和 TF 同一条运动链。"""
        return self.arm.pose(ARM)[0].tolist()

    def plan_left_to(self, target):
        plan = self.arm.plan_cartesian(ARM, np.array(target, dtype=float))    # 姿态保持不变
        if isinstance(plan, str):
            return {"ok": False, "error": plan}
        slow_down(plan, self.speed)
        return {"ok": True, "plan": plan, "summary": plan.summary()}

    def execute(self, plan):
        ok, msg = self.arm.execute(plan["plan"])
        return {"ok": ok, "msg": msg}

    def close(self):
        self.arm.cancel()
        self.camera.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.arm.destroy_node()
        self.node.destroy_node()
        rclpy.try_shutdown()
