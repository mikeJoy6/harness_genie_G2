"""末端六维力:GDK /gdk/motion_control_status 的 frame_wrenchs(100Hz,arm_l_end_link / arm_r_end_link)。

给 touch_object 最后一段(只放过物体附近的点云)用:力的变化超过阈值 = 碰到东西了(桌面、物体),马上停。
照搬 ~/agi@g2_robot/src/task_board_pose/task_board_pose/motion/wrench.py(纸板任务的“压到东西就停”,真机用过)。

已知(agi@g2_robot 的 arm_monitor.py,2026-11-02 实测):
  · 力在末端 end_link 系,GDK 已减掉手臂自身重力,但不知道末端执行器(夹爪重量会算成外力)
    → 每次用前先在静止时取平均当零点(tare),之后只看“比零点变了多少”。
  · 手上没东西时 6 路严格等于 0,有死区:很小的力显示为 0。
  · 官方量程没确认(G1 是 200N / 7Nm)。
"""
import threading
import time

import numpy as np

TOPIC = "/gdk/motion_control_status"


def force_change_n(force, force_zero):
    """力(前 3 维,N)比零点变了多少,取模长。方向不管:桌面从下面顶、物体从侧面挡都算。"""
    change = np.asarray(force[:3], dtype=float) - np.asarray(force_zero[:3], dtype=float)
    return float(np.linalg.norm(change))


class GdkWrench:
    def __init__(self, sensor_node):
        """sensor_node:harness/perception/sensor_node.py 的 SensorNode,用时才订,用完就撤。"""
        self.sensor = sensor_node
        self.lock = threading.Lock()
        self.latest_by_frame = {}           # frame → (6 维 [fx fy fz mx my mz], 收到时刻)

    def start(self, wait_s=1.0):
        """开始订阅;wait_s 内收到第一帧返回 True。"""
        from gdk_msgs.msg import MotionControlStatus
        from rclpy.qos import qos_profile_sensor_data
        with self.lock:
            self.latest_by_frame = {}
        self.sensor.open([(MotionControlStatus, TOPIC, self._on_status, qos_profile_sensor_data)])
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            with self.lock:
                if self.latest_by_frame:
                    return True
            time.sleep(0.02)
        return False

    def stop(self):
        self.sensor.close()

    def _on_status(self, msg):
        now = time.monotonic()
        with self.lock:
            for name, w in zip(msg.frame_names, msg.frame_wrenchs):
                values = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]
                self.latest_by_frame[name] = (np.array(values), now)

    def latest(self, frame, max_age_s=0.1):
        """最新 6 维力。没有、或超过 max_age_s 没更新就抛 RuntimeError(看不见力就不能往前走)。"""
        with self.lock:
            got = self.latest_by_frame.get(frame)
        if got is None or time.monotonic() - got[1] > max_age_s:
            raise RuntimeError(f"六维力 {frame} {max_age_s}s 没更新({TOPIC})")
        return got[0].copy()

    def average(self, frame, duration_s, max_age_s=0.1):
        """静止时取平均当零点。"""
        samples = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            samples.append(self.latest(frame, max_age_s))
            time.sleep(0.01)
        return np.mean(samples, axis=0)
