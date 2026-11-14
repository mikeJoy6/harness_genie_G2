"""真机器人:手臂运控(harness/motion/)+ 头部 RGBD(harness/perception/),都是本项目里的代码。

  · 手臂末端:ArmMotion —— 先积分笛卡尔直线,会撞再 plan_detour(via_then_rrt,和 pose_ab_search 同一套)
  · 关节组(腰/头/双臂):WholeBody —— 关节直线插值 + 整机碰撞检查,会撞再 JointDetour(经录好位姿中转 / 关节 RRT)
  · touch_object:plan_touch —— 唯一可以不查点云碰撞的运动,固定两段(查碰撞到预接触点 → 不查点云慢速直线贴上去)
  · 夹爪:GdkGripper(GDK 伺服话题)

运行前要 source ~/agi@g2_robot/env_g2.sh(GDK 的 DDS 配置),并且 hardware.launch.py 已经在跑。
"""
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from ..geometry import matrix_to_rpy_deg, rpy_deg_to_matrix
from ..motion.arm_motion import CART_SPEED, ArmMotion
from ..motion.goal_pose import ComfortPose, body_assist_search, free_goal, roll_locked, tilt_search
from ..motion.joint_detour import JointDetour
from ..motion.whole_body import WholeBody
from ..perception.calib import CALIB
from ..perception.head_aim import HEAD_JOINTS, HeadAim
from ..perception.rgbd_source import RosSource
from ..perception.sam3_client import Sam3Client
from ..perception.sam3_objects import objects_from_masks, public, save_segmentation
from ..perception.scene3d import Rgbd
from ..perception.sensor_node import SensorNode
from ..perception.wrist_camera import WristCamera
from ..touch_keep_out import gripper_hits, outside_object, rests_on_surface
from .gripper_gdk import GdkGripper
from .wrench_gdk import GdkWrench, force_change_n


def _angle_deg(a, b):
    """两个方向之间的夹角(度)。"""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    cos = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


class ArmGoal:
    """一次 plan_arm_to 的参数打包,每个步骤(_step_*)都读它,不用一长串参数传来传去。"""
    def __init__(self, arm, target, rpy_deg, orientation, roll_deg, tool_axis, retreat):
        self.arm = arm
        self.target = np.array(target, dtype=float)
        self.rpy_deg = rpy_deg
        self.orientation = tuple(orientation)
        self.roll_deg = roll_deg
        self.tool_axis = tool_axis
        self.retreat = retreat


ORIENTATION_TEXT = {"exact": "按指定", "axis": "工具轴指定方向", "keep": "保持当前", "comfort": "舒适", "free": "放松"}


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
    def __init__(self, vision_cfg, speed_m_s, debug_dir="traces", poses=None, gripper_cfg=None, touch_cfg=None,
                 tool_axis_tolerance_deg=15.0, body_assist_cfg=None):
        self.vision = vision_cfg
        self.speed = speed_m_s
        self.debug_dir = Path(debug_dir)
        self.poses = poses or {}            # JointDetour 的中转候选:{位姿名: {关节名: 弧度}}
        self.rgbd = Rgbd()
        self._limits = None
        self._head_aim = None                # 第一次 head_angles_to 时用 URDF 建
        self._comfort = {}                   # 每条手臂一个 ComfortPose,第一次用时建
        self.touch = touch_cfg               # touch_object 的链路参数(harness.yaml motion.touch)
        self.tool_axis_tolerance_deg = tool_axis_tolerance_deg   # tool_axis 严格做不到时,差这么多度以内也接受
        self.body_assist = body_assist_cfg   # 手臂够不着时先动腰(前倾 / 转腰),None = 不用
        sam3 = vision_cfg["sam3"]
        self.sam3 = Sam3Client(sam3["url"], timeout_s=sam3["timeout_s"])

        rclpy.init()
        self.node = rclpy.create_node("robot_harness")
        self.arm = ArmMotion("robot_harness_arm")
        self.body = WholeBody(self.node)        # 订阅 /joint_states,连 4 个 JTC 和 /check_state_validity
        self.camera = SensorNode("robot_harness_camera")
        self.gripper = GdkGripper(self.node, gripper_cfg) if gripper_cfg else None   # 第一次用时才连 GDK
        self.wrist = None                    # 腕部相机(用时才订),harness.yaml vision.wrist
        if vision_cfg.get("wrist"):
            self.wrist = WristCamera(SensorNode("robot_harness_wrist"), vision_cfg["wrist"])
        self.wrench = None                   # touch 最后一段的六维力(用时才订 /gdk/motion_control_status)
        if touch_cfg and touch_cfg.get("force", {}).get("enabled"):
            self.wrench = GdkWrench(SensorNode("robot_harness_wrench"))

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

    # ---------------- SAM 3:按名字找物体 ----------------
    def _grab_frame(self, timeout_s):
        """取一帧同步的 (彩色 BGR, 深度 mm, base_link→head_link3);超时返回 None。"""
        source = RosSource(sensor=self.camera)
        deadline = time.monotonic() + timeout_s
        try:
            while time.monotonic() < deadline:
                frame = source.next(None)
                if frame is not None:
                    return frame[:3]
                time.sleep(0.01)
        finally:
            source.close()
        return None

    def find_objects(self, query, suffix=""):
        """一帧画面 → SAM 3 找所有 query → 每个掩码 + 深度 → base_link 坐标,按离相机由近到远编号。
        每次都把画面、掩码、结果存到 traces/sam3/<时间>_<短语><suffix>/。"""
        cfg = self.vision["sam3"]
        frame = self._grab_frame(self.vision["timeout_s"])
        if frame is None:
            return {"ok": False, "error": "没有收到头部相机画面(/gdk/camera/head_color、head_depth、/gdk/tf)"}
        bgr, depth_mm, T_base_head = frame
        t0 = time.monotonic()
        try:
            results = self.sam3.segment(bgr, [query], cfg["threshold"])
        except RuntimeError as e:
            return {"ok": False, "error": str(e)}
        segment_ms = round((time.monotonic() - t0) * 1000)

        depth_m = depth_mm.astype(np.float32) / 1000.0
        points, cam_xyz = self.rgbd.points_base(depth_m, T_base_head)
        principal = (CALIB["depth"]["cx"], CALIB["depth"]["cy"])
        objects, rejected = objects_from_masks(results, depth_m, points, cam_xyz, principal,
                                               cfg["min_valid_depth_pixels"])
        meta = {"threshold": cfg["threshold"], "segment_ms": segment_ms, "sam3_timing_ms": self.sam3.last_timing_ms,
                "camera_xyz_m": [round(float(v), 4) for v in cam_xyz],
                "base_link_T_head_link3": np.round(T_base_head, 6).tolist(),
                "head_joints_deg": {j: round(float(np.degrees(self.body.snapshot()[j])), 2) for j in HEAD_JOINTS},
                "rejected": rejected}
        folder = save_segmentation(self.debug_dir / "sam3", query, bgr, depth_mm, results, objects, meta, suffix)
        return {"ok": True, "query": query, "objects": [public(o) for o in objects],
                "rejected": rejected[:3], "log_dir": str(folder)}

    def head_angles_to(self, xyz):
        """头部关节角:让相机光轴对准 xyz(base_link)。只改 yaw、pitch,夹在 URDF 限位里。"""
        if self._head_aim is None:
            self._head_aim = HeadAim(self.arm.urdf)
        target, residual, at_limit = self._head_aim.solve(self.joint_state(), xyz, self.joint_limits())
        return {"ok": True, "target": target, "residual_deg": residual, "at_limit": at_limit}

    # ---------------- 拍照 ----------------
    def take_photo(self, camera, folder):
        """从相机话题取一帧 JPEG,原样存到 folder/<时间>_<相机>.jpg。机器人不动。"""
        cfg = self.vision["photo"]
        topic = cfg["cameras"].get(camera)
        if topic is None:
            return {"ok": False, "error": f"没有相机 '{camera}',可选:{list(cfg['cameras'])}"}
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        got = []

        def on_image(msg):
            if not got:
                got.append(msg)

        sub = self.node.create_subscription(CompressedImage, topic, on_image, qos_profile_sensor_data)
        try:
            deadline = time.monotonic() + cfg["timeout_s"]
            while not got and time.monotonic() < deadline:
                time.sleep(0.02)
        finally:
            self.node.destroy_subscription(sub)
        if not got:
            return {"ok": False, "error": f"{cfg['timeout_s']}s 内没收到 {topic} 的画面"}
        data = bytes(got[0].data)
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return {"ok": False, "error": f"{topic} 的数据解不开(format={got[0].format})"}
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{time.strftime('%Y%m%d_%H%M%S')}_{camera}.jpg"
        path.write_bytes(data)
        return {"ok": True, "path": str(path), "camera": camera, "width": image.shape[1], "height": image.shape[0]}

    def wrist_frame(self, arm):
        """腕部相机取一帧:{"ok", "bgr", "depth_m"(没有深度时 None)}。还没有工具调用它,对准检查做好后再接
        (设计见 docs/wrist_camera.md);现在可以用 take_photo(camera="hand_right") 存一张看。"""
        if self.wrist is None:
            return {"ok": False, "error": "没有配置腕部相机(harness.yaml vision.wrist)"}
        return self.wrist.grab(arm)

    # ---------------- 夹爪 ----------------
    def gripper_state(self, arm):
        if self.gripper is None:
            return {"ok": False, "arm": arm, "error": "没有配置夹爪(harness.yaml 的 motion.gripper)"}
        return self.gripper.state(arm)

    def gripper_command(self, arm, action):
        if self.gripper is None:
            return {"ok": False, "arm": arm, "error": "没有配置夹爪(harness.yaml 的 motion.gripper)"}
        return self.gripper.command(arm, action)

    # ---------------- 读 ----------------
    def end_pose(self, arm):
        """手臂链末端(有 TCP 就是 tcp_*_link = 夹爪尖,否则 arm_*_end_link)在 base_link 下的位姿,
        由实测关节角正运动学得到,和 TF 同一条运动链。"""
        p, R = self.arm.pose(arm)
        return {"xyz_m": p.tolist(), "rpy_deg": matrix_to_rpy_deg(R), "link": self.arm.kins[arm].tip}

    def joint_state(self):
        return self.body.snapshot()

    def joint_limits(self):
        """URDF 里每个关节的 (下限, 上限),弧度。第一次调用时解析,之后复用。"""
        if self._limits is None:
            from urdf_parser_py.urdf import URDF
            robot = URDF.from_xml_string(self.arm.urdf)
            self._limits = {j.name: (j.limit.lower, j.limit.upper) for j in robot.joints
                            if j.limit is not None and j.type != "continuous"}
        return dict(self._limits)

    # ---------------- 规划(不动) ----------------
    def plan_arm_to(self, arm, target, rpy_deg=None, orientation=("keep",), wrist_roll_deg=0.0, tool_axis=None,
                    retreat=None, allow_body=True):
        """末端到 target(base_link,米),全程查碰撞(含点云)。按顺序试下面几步,第一个成功的就用:
          1. _step_orientations   终点姿态:rpy_deg 给了就按它;tool_axis 给了就工具轴朝它(roll 锁住);
                                  否则按 orientation 的顺序试 keep / comfort / free。每种都先直线、再 via_then_rrt 绕障
          2. _step_tilt_here      (给了 tool_axis)同一位置,工具轴在容差内最接近要求的方向
          3. _step_tilt_retreat   (给了 tool_axis 和 retreat)往身体方向后撤着转
          4. _step_body_assist    (allow_body 且配置打开)手臂够不着:先前倾 / 转腰,到位后再规划手臂
        要碰到物体(目标本身在点云里)不走这里,走 plan_touch。"""
        goal = ArmGoal(arm, target, rpy_deg, orientation, wrist_roll_deg, tool_axis, retreat)
        steps = [self._step_orientations]
        if tool_axis is not None:
            steps.append(self._step_tilt_here)
            if retreat:
                steps.append(self._step_tilt_retreat)
        if allow_body and self.body_assist and self.body_assist.get("enabled"):
            steps.append(self._step_body_assist)
        failures = []
        for step in steps:
            plan = step(goal, failures)
            if plan is not None:
                return plan
        return {"ok": False, "error": " | ".join(failures)}

    def _line_or_detour(self, arm, p, R):
        """直线(积分笛卡尔)→ 不行再末端绕障(via_then_rrt)。返回 (Plan, None) 或 (None, 两次失败的原因)。"""
        plan = self.arm.plan_cartesian(arm, p, R)
        if isinstance(plan, str):
            detour = self.arm.plan_detour(arm, p, R)
            if isinstance(detour, str):
                return None, f"直线 {plan};绕障 {detour}"
            plan = detour
        slow_down(plan, self.speed)
        return plan, None

    def _step_orientations(self, goal, failures):
        if goal.rpy_deg is not None:
            strategies = ("exact",)
        elif goal.tool_axis is not None:
            strategies = ("axis",)                  # 工具轴指定方向,roll 锁住
        else:
            strategies = goal.orientation
        for how in strategies:
            R, note = self._goal_rotation(goal.arm, goal.target, how, goal.rpy_deg, goal.roll_deg, goal.tool_axis)
            if R is None and how != "keep":
                failures.append(f"{how}:{note}")
                continue
            plan, why = self._line_or_detour(goal.arm, goal.target, R)
            if plan is None:
                failures.append(f"{how}:{why}")
                continue
            return {"ok": True, "kind": "arm", "plan": plan, "orientation": how, "orientation_note": note,
                    "summary": f"终点姿态 {ORIENTATION_TEXT[how]}{note};{plan.summary()}",
                    "lowest_z_m": self._lowest_z(goal.arm, plan.path)}
        return None

    def _step_tilt_here(self, goal, failures):
        """原地严格朝 tool_axis 做不到:同一个位置找最接近的朝向(差 ≤ tool_axis_tolerance_deg 就接受)。"""
        return self._tilt(goal, {"max_m": 0.0, "step_m": 1.0}, failures, self.tool_axis_tolerance_deg)

    def _step_tilt_retreat(self, goal, failures):
        return self._tilt(goal, goal.retreat, failures, None)

    def _step_body_assist(self, goal, failures):
        after = {"kind": "arm", "args": {"arm": goal.arm, "target": goal.target.tolist(), "rpy_deg": goal.rpy_deg,
                                         "orientation": goal.orientation, "wrist_roll_deg": goal.roll_deg,
                                         "tool_axis": goal.tool_axis}}
        return self._body_assist(goal.arm, goal.target, goal.tool_axis, goal.roll_deg, failures, after)

    def _tilt(self, goal, retreat, failures, max_miss_deg):
        """原地转不到 tool_axis:沿“末端 → 肩”的水平方向后撤,每步先转到能转的最大程度(goal_pose.tilt_search),
        找到撤得最少、转得最多的位姿,再一次规划过去(直线 → via_then_rrt)。失败返回 None 并记下原因。"""
        arm, target, tool_axis = goal.arm, goal.target, goal.tool_axis
        pb, Rb = self.arm._root_in_base()
        solver = self.arm.solvers[arm]
        q0 = self.arm.q_arm(arm)
        R_now = solver.kin.fk(q0)[1]
        back = np.array(self.shoulder_xyz(arm)) - target
        back[2] = 0.0
        if np.linalg.norm(back) < 1e-6:
            failures.append("后撤:末端就在肩正下方,不知道往哪撤")
            return None
        back = back / np.linalg.norm(back)
        found = tilt_search(solver, q0, Rb.T @ (target - pb), R_now, Rb.T @ np.asarray(tool_axis, float),
                            Rb.T @ back, retreat["max_m"], retreat["step_m"], goal.roll_deg)
        if found is None:
            failures.append(f"后撤 {retreat['max_m'] * 100:.0f}cm 内一点都转不动")
            return None
        final = pb + Rb @ found["p"]
        R_goal = Rb @ found["R"]
        miss = _angle_deg(R_goal[:, 2], tool_axis)
        if max_miss_deg is not None and miss > max_miss_deg:
            failures.append(f"原位置最多转到离目标方向 {miss:.0f}°(允许 {max_miss_deg:.0f}°)")
            return None
        plan, why = self._line_or_detour(arm, final, R_goal)
        if plan is None:
            failures.append(f"后撤 {found['retreat_m'] * 100:.0f}cm 转 {found['fraction'] * 100:.0f}% 有 IK 解,"
                            f"但规划不过去:{why}")
            return None
        if found["fraction"] >= 1.0:
            how = "转到位"
        else:
            how = f"离目标方向还差 {miss:.0f}°"
        if found["retreat_m"] > 0:
            what = f"原地转不到,后撤 {found['retreat_m'] * 100:.0f}cm 再转,{how}"
        else:
            what = f"严格朝向够不着,{how}"
        return {"ok": True, "kind": "arm", "plan": plan, "orientation": "axis",
                "final_target": final.tolist(), "retreat_m": found["retreat_m"], "tilt_fraction": found["fraction"],
                "tool_axis_miss_deg": round(miss, 1),
                "summary": f"{what};{plan.summary()}",
                "lowest_z_m": self._lowest_z(arm, plan.path)}

    def _body_assist(self, arm, target, tool_axis, roll_deg, failures, after):
        """手臂在现在的腰姿态下够不着 target:找改动最小的前倾 / 转腰(goal_pose.body_assist_search,只看 IK),
        规划腰的运动(整机碰撞检查)。after = 腰到位后要重新规划的东西({"kind": "arm" 或 "touch", "args"}),
        在 _execute_body_then 里做,那时不再动腰。失败返回 None 并记下原因。"""
        cfg = self.body_assist
        names = self.arm.bkin.joint_names
        joints = self.joint_state()
        q_body = np.array([joints[n] for n in names])
        limits = self.joint_limits()
        body_limits = [(limits[n][0] + 0.02, limits[n][1] - 0.02) for n in names]
        lean = [d for d in (5, 10, 15, 20, 25, 30, 35, 40) if d <= cfg["max_lean_deg"]]
        turn = [d for d in (0, 10, -10, 20, -20, 30, -30) if abs(d) <= cfg["max_turn_deg"]]
        found = body_assist_search(self.arm.bkin, self.arm.solvers[arm], q_body, self.arm.q_arm(arm), target,
                                   tool_axis, self.tool_axis_tolerance_deg, roll_deg, lean, turn, body_limits)
        if found is None:
            failures.append(f"动腰(前倾 ≤{cfg['max_lean_deg']}°、转腰 ≤{cfg['max_turn_deg']}°)也够不着")
            return None
        body_plan = self.plan_joints({n: float(v) for n, v in zip(names, found["q_body"])}, cfg["speed_rad_s"])
        if not body_plan["ok"]:
            failures.append(f"前倾 {found['lean_deg']}° 转腰 {found['turn_deg']}° 就够得着,"
                            f"但腰的运动规划不过去:{body_plan['error']}")
            return None
        return {"ok": True, "kind": "body_then", "arm": arm, "body": body_plan, "after": after,
                "lean_deg": found["lean_deg"], "turn_deg": found["turn_deg"],
                "summary": f"手臂够不着,先上身前倾 {found['lean_deg']}°、转腰 {found['turn_deg']}°"
                           f"({body_plan['summary']}),再从新姿态规划手臂",
                "lowest_z_m": float(target[2])}

    # ---------------- touch_object:唯一可以不查点云碰撞的运动 ----------------
    def plan_touch(self, arm, object_xyz, gap_m, direction, allow_body=True):
        """夹爪尖碰到(或停在 gap_m 外)物体。物体本身在点云里,直接规划到它跟前必然“会撞”,所以固定走两段:

          接触点 C = 物体 − gap·d(d = 接近方向,夹爪尖朝 d,roll 锁住)
          预接触点 P = C − b·d,b 从近到远(touch.backoffs_cm)
          ⓪ 先在接触点 C 定夹爪姿态和关节角(_touch_contact_pose:朝 d,严格不行就容差内最接近;只要求不自碰撞)
          ① 对每个 P 离线验(便宜):从 C 的关节角沿直线反着走到 P —— P、C 在同一个构型分支,P → C 直线一定走得通;
             P 处不撞(含点云);整段 MoveIt 只放过点云,自碰撞照查;
             点云再用几何查一遍(_keep_out_ok):只有物体附近的格可以碰
          ② 验过的 P 才规划第一段(全程查碰撞):直线 → 末端绕障 via_then_rrt → 关节空间直接去 P 的关节角
             (经录好位姿中转 / RRT)。终点构型已经验证过不撞,只要连得通,关节空间就找得到 ——“能去就一定规划得出来”
          ③ 执行时到了 P,再从实际位姿慢速直线到 C(_execute_touch),执行前用最新点云再查一次 ①的几何检查
        接近方向依次试:末端 → 物体(direction)、肩 → 物体水平(物体放在台面上时不用)、从上往下。
        某个方向连接触点都够不着时,先动腰再试**这个方向**(allow_body),不直接换方向
        —— 2026-11-14 真机:朝下够不着就换成水平平推,贴着桌面撞上去了。"""
        cfg = self.touch
        obj = np.asarray(object_xyz, dtype=float)
        start = np.array(self.end_pose(arm)["xyz_m"])
        d0 = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
        contact0 = obj - gap_m * d0
        near = float(np.linalg.norm(contact0 - start))
        if near <= cfg["backoffs_cm"][0] / 100:      # 已经在跟前:只走最后一段(执行前照样做几何检查)
            return {"ok": True, "kind": "touch", "arm": arm, "first": None, "contact_xyz_m": contact0.tolist(),
                    "object_xyz_m": obj.tolist(), "precontact_xyz_m": start.tolist(), "unchecked_m": round(near, 3),
                    "approach": "末端→物体", "direction_notes": [], "direction": d0.tolist(), "gap_m": gap_m,
                    "summary": f"已经在接触点 {near * 100:.0f}cm 内,直接慢速直线过去"}

        cloud = self.arm.octomap_base()
        blocked = outside_object(cloud, obj, cfg["keep_out"]["object_radius_m"])
        # 要碰上(gap 0)且有六维力时,执行会越过接触点再走 overshoot_m、碰到才停(_execute_touch)。
        #   所以姿态、关节角、动腰都按最深那一点定,不然手臂在接触点就快伸直了,压不下去(离线踩过:只剩 1.5cm)
        depth = 0.0
        if gap_m == 0 and self.wrench is not None:
            depth = cfg["force"]["overshoot_m"]
        notes = []                                    # 每个没用上的方向为什么没用上,写进结果
        failures = []
        tried = 0
        for name, d in self._touch_directions(arm, obj, d0, cloud, notes):
            contact = obj - gap_m * d
            deepest = contact + depth * d                  # 最后一段最远可能走到哪
            rolls = []                                    # [(roll, 最深点姿态 + 关节角)],够不着的 roll 不要
            for roll in cfg["roll_offsets_deg"]:
                contact_pose = self._touch_contact_pose(arm, deepest, d, roll)
                if contact_pose is not None:
                    rolls.append((roll, contact_pose))
            if not rolls:
                where = "接触点" if depth == 0 else f"接触点再往前 {depth * 100:.0f}cm(碰到才停,要留出余量)"
                why = f"{name}:{where}够不着(朝向差超过 {self.tool_axis_tolerance_deg:.0f}°)"
                if allow_body and self.body_assist and self.body_assist.get("enabled"):
                    after = {"kind": "touch", "args": {"arm": arm, "object_xyz": obj.tolist(), "gap_m": gap_m,
                                                       "direction": d.tolist()}}
                    body = self._body_assist(arm, deepest, d.tolist(), 0.0, failures, after)
                    if body is not None:
                        body["summary"] = f"{why},{body['summary']}"
                        body["direction_notes"] = notes + [why + ",先动腰"]
                        return body
                    why += ",动腰也够不着"
                notes.append(why)
                continue
            counts = {"ik": 0, "collide": 0, "last": 0, "keep_out": 0}
            for b_cm in cfg["backoffs_cm"]:
                pre = contact - b_cm / 100 * d
                for roll, contact_pose in rolls:
                    cand = self._touch_candidate(arm, pre, contact_pose, blocked, contact, d)
                    if isinstance(cand, str):
                        counts[cand] += 1
                        continue
                    tried += 1
                    first, why = self._reach_precontact(arm, pre, cand, deepest, blocked, contact, d)
                    if first is not None and first.get("shallow") and allow_body and self.body_assist \
                            and self.body_assist.get("enabled"):
                        # 够得着接触点,但压不到“接触点 + overshoot”(碰不到桌面):先试多前倾,不行再接受浅的
                        after = {"kind": "touch", "args": {"arm": arm, "object_xyz": obj.tolist(), "gap_m": gap_m,
                                                           "direction": d.tolist()}}
                        body = self._body_assist_lean_more(arm, deepest, d, cand["R"], failures, after)
                        if body is not None:
                            body["summary"] = f"{name}:现在的姿态越过接触点压不到 {depth * 100:.0f}cm,{body['summary']}"
                            body["direction_notes"] = notes
                            return body
                    if first is not None:
                        if first.get("shallow"):
                            notes.append(f"{name}:这个姿态越过接触点压不到 {depth * 100:.0f}cm,执行时能压多深压多深")
                        return {"ok": True, "kind": "touch", "arm": arm, "first": first,
                                "contact_xyz_m": contact.tolist(), "object_xyz_m": obj.tolist(),
                                "precontact_xyz_m": pre.tolist(),
                                "unchecked_m": b_cm / 100, "approach": name, "roll_offset_deg": roll,
                                "direction": d.tolist(), "gap_m": gap_m,
                                "direction_notes": notes,
                                "summary": f"接近方向 {name};① 查碰撞到接触点前 {b_cm}cm:{first['summary']};"
                                           f"② 只放过物体附近的点云,直线 {b_cm}cm 到接触点({cfg['final_speed_m_s']} m/s)"}
                    failures.append(f"{name} {b_cm}cm:第一段规划不过去({why})")
                    break                    # 这个 b 规划不通,换下一个 b(别的 roll 多半一样)
                if tried >= cfg["max_plan_tries"]:
                    failures.append(f"已试 {tried} 个预接触点,停止")
                    break
            notes.append(f"{name}:预接触点 {counts['ik']} 个 IK 无解、{counts['collide']} 个会撞、"
                         f"{counts['last']} 个最后一段走不通或自碰撞、{counts['keep_out']} 个最后一段会碰到物体以外的点云")
            swing = [f for f in failures if f.startswith(name) and "不走" in f]
            if swing and allow_body and self.body_assist and self.body_assist.get("enabled"):
                # 现在的手臂构型够不着、换构型又要大幅甩臂:先动腰,让现在这个构型够得着
                after = {"kind": "touch", "args": {"arm": arm, "object_xyz": obj.tolist(), "gap_m": gap_m,
                                                   "direction": d.tolist()}}
                R_goal = roll_locked(self.arm.pose(arm)[1], d, 0.0)
                body = self._body_assist_lean_more(arm, deepest, d, R_goal, failures, after)
                if body is not None:
                    body["summary"] = f"{name}:手臂要大幅换构型才够得着,{body['summary']}"
                    body["direction_notes"] = notes
                    return body
            if tried >= cfg["max_plan_tries"]:
                break
        return {"ok": False, "error": " | ".join(notes + failures)}

    def _body_assist_lean_more(self, arm, target, d, R_goal, failures, after):
        """现在的手臂构型“够是够得着,但要换构型 / 压不够深”时,多前倾一点让**现在这个构型**够得着 target。
        body_assist_search 只看 IK 有没有解(哪个构型都算),前倾后可能还是要换构型,所以这里自己验:
        腰换成候选姿态、手臂关节不动,从现在的关节角沿直线积分到 target(姿态 R_goal,base_link),到得了才算。
        动腰后整个 plan_touch 重来(不再动腰)。失败返回 None。"""
        cfg = self.body_assist
        names = self.arm.bkin.joint_names
        joints = self.joint_state()
        q_body = np.array([joints[n] for n in names])
        limits = self.joint_limits()
        solver = self.arm.solvers[arm]
        q_arm = np.asarray(self.arm.q_arm(arm), dtype=float)
        found = None
        for lean in (5, 10, 15, 20):
            if lean > cfg["max_lean_deg"] or found is not None:
                continue
            for turn in (0, 10, -10):
                if abs(turn) > cfg["max_turn_deg"] or found is not None:
                    continue
                q = q_body.copy()
                q[2] += math.radians(lean)
                q[4] += math.radians(turn)
                inside = True
                for n, v in zip(names, q):
                    if not limits[n][0] + 0.02 <= v <= limits[n][1] - 0.02:
                        inside = False
                if not inside:
                    continue
                pb, Rb = self.arm.bkin.fk(q)
                R_root = Rb.T @ R_goal
                _, reached, _, _, _ = solver.solve(q_arm, Rb.T @ (np.asarray(target) - pb), "full",
                                                   R_root, R_root[:, 0])
                if reached:
                    found = (q, lean, turn)
        if found is None:
            failures.append("多前倾(现在的手臂构型)也够不着")
            return None
        q, lean, turn = found
        body_plan = self.plan_joints({n: float(v) for n, v in zip(names, q)}, cfg["speed_rad_s"])
        if not body_plan["ok"]:
            failures.append(f"多前倾 {lean}° 的腰运动规划不过去:{body_plan['error']}")
            return None
        return {"ok": True, "kind": "body_then", "arm": arm, "body": body_plan, "after": after,
                "lean_deg": lean, "turn_deg": turn,
                "summary": f"先上身再前倾 {lean}°、转腰 {turn}°({body_plan['summary']}),再从新姿态规划手臂",
                "lowest_z_m": float(target[2])}

    def _touch_directions(self, arm, obj, direction, cloud, notes):
        """接近方向候选(单位向量,从手指向物体),彼此差 15° 以上才算不同。
        物体放在台面上(rests_on_surface)时不用“肩 → 物体水平”:那是贴着台面平推。"""
        cands = [("末端→物体", direction)]
        horizontal = obj - np.array(self.shoulder_xyz(arm))
        horizontal[2] = 0.0
        if np.linalg.norm(horizontal) > 1e-6:
            if rests_on_surface(cloud, obj):
                notes.append("肩→物体水平:物体放在台面上,不从侧面贴着台面平推")
            else:
                cands.append(("肩→物体水平", horizontal))
        cands.append(("从上往下", np.array([0.0, 0.0, -1.0])))
        result = []
        for name, d in cands:
            d = np.asarray(d, dtype=float) / np.linalg.norm(d)
            if all(_angle_deg(d, e) > 15.0 for _, e in result):
                result.append((name, d))
        return result

    def _keep_out_ok(self, arm, path, blocked, contact, d):
        """最后一段(MoveIt 不查点云)的几何检查:夹爪近似成胶囊,沿 path 每个关节角算夹爪位置,
        碰到 blocked(物体附近以外的点云格)就不行。path 是这条手臂的关节角序列。
        只查到接触点为止(夹爪尖还没越过“过接触点、垂直于 d 的平面”的那些位姿):
        越过接触点的那段是靠六维力“碰到就停”的,目标本来就在桌面里 ——
        2026-11-14 真机:连这段也查,桌面(背面补全成好几层厚)把最后一段全拦了,只好去绕 154° 的关节空间。"""
        if len(blocked) == 0:
            return True
        cfg = self.touch["keep_out"]
        pb, Rb = self.arm._root_in_base()
        kin = self.arm.solvers[arm].kin
        P = np.asarray(path, dtype=float)
        dense = [P[0]]
        for q0, q1 in zip(P[:-1], P[1:]):                # 相邻路点之间按 1° 插值,别跳过薄薄一层桌面
            n = max(1, int(np.ceil(np.max(np.abs(q1 - q0)) / np.radians(1.0))))
            for s in np.linspace(0.0, 1.0, n + 1)[1:]:
                dense.append(q0 + s * (q1 - q0))
        contact = np.asarray(contact, dtype=float)
        d = np.asarray(d, dtype=float)
        tips = []
        axes = []
        for q in dense:
            p, R = kin.fk(q)
            tip = pb + Rb @ p
            if float((tip - contact) @ d) > 0.005:          # 已经越过接触点:这段归六维力管
                continue
            tips.append(tip)
            axes.append(Rb @ R[:, 2])
        if not tips:
            return True
        hits = gripper_hits(tips, axes, blocked, cfg["gripper_radius_m"], cfg["gripper_length_m"])
        return hits == 0

    def _touch_contact_pose(self, arm, contact, d, roll_deg):
        """接触点处的夹爪姿态和关节角。工具轴朝 d、roll 锁住(+roll_deg);严格做不到时接受离 d
        不超过 tool_axis_tolerance_deg 的朝向。关节角只要求不自碰撞(接触点本来就在点云里)。
        最后一段直线全程保持这个姿态,所以从最难够的接触点定。够不着返回 None。"""
        pb, Rb = self.arm._root_in_base()
        solver = self.arm.solvers[arm]
        q_now = self.arm.q_arm(arm)
        R_now_root = solver.kin.fk(q_now)[1]
        d_root = Rb.T @ d
        p_contact = Rb.T @ (contact - pb)
        R_root = Rb.T @ roll_locked(Rb @ R_now_root, d, roll_deg)
        if solver.ik(p_contact, "full", R_root, None, q_now) is None:
            near = tilt_search(solver, q_now, p_contact, R_now_root, d_root, d_root, 0.0, 1.0, roll_deg)
            if near is None or _angle_deg(near["R"][:, 2], d_root) > self.tool_axis_tolerance_deg:
                return None
            R_root = near["R"]

        def no_self_collision(q):
            return self.arm.valid(arm, q, ignore_cloud=True)

        q_contact = solver.ik(p_contact, "full", R_root, None, q_now, valid=no_self_collision)
        if q_contact is None:
            return None
        return {"R": Rb @ R_root, "q": q_contact}

    def _touch_candidate(self, arm, pre, contact_pose, blocked, contact, d):
        """离线验一个预接触点:从接触点的关节角沿直线反着走到 P(同一个构型分支,保证 P → C 直线走得通),
        P 处不撞(含点云),整段除了点云不撞,而且夹爪不碰物体附近以外的点云格(blocked)。
        返回 {"q", "R"} 或失败种类 "ik" / "collide" / "last" / "keep_out"。"""
        pb, Rb = self.arm._root_in_base()
        solver = self.arm.solvers[arm]
        R_root = Rb.T @ contact_pose["R"]
        path, reached, _, _, _ = solver.solve(contact_pose["q"], Rb.T @ (pre - pb), "full", R_root, R_root[:, 0])
        if not reached:
            return "ik"
        q_pre = np.asarray(path[-1], dtype=float)
        if not self.arm.valid(arm, q_pre):
            return "collide"
        if not self._keep_out_ok(arm, path, blocked, contact, d):
            return "keep_out"
        if not self.arm.path_clear(arm, path[::-1], ignore_cloud=True):
            return "last"
        return {"q": q_pre, "R": contact_pose["R"]}

    def _final_segment_ok(self, arm, q, target, R, blocked, contact, d):
        """从关节角 q、保持姿态 R,积分笛卡尔直线能不能到 target(接触点,或越过它的最深点),
        且除了点云不撞、到接触点为止不碰物体附近以外的点云格。"""
        pb, Rb = self.arm._root_in_base()
        solver = self.arm.solvers[arm]
        R_root = Rb.T @ R
        path, reached, _, _, _ = solver.solve(np.asarray(q, dtype=float), Rb.T @ (target - pb), "full",
                                              R_root, R_root[:, 0])
        return reached and self._keep_out_ok(arm, path, blocked, contact, d) \
            and self.arm.path_clear(arm, path, ignore_cloud=True)

    def _reach_precontact(self, arm, pre, cand, target, blocked, contact, d):
        """第一段(全程查碰撞,含点云)到预接触点:直线 → 末端绕障 → 关节空间(经录好位姿中转 / RRT)。
        前两种按末端位姿规划,到了以后的构型分支可能和验过的不一样:要再验一次“从这里直线下得去”,
        不行就改关节空间直接去验过的那个关节角。返回 (计划, None) 或 (None, 原因)。"""
        plan, why = self._line_or_detour(arm, pre, cand["R"])
        if plan is not None:
            # 只要求从这个构型直线到得了接触点;越过接触点能多走多少,执行时按这个构型试(_execute_touch)。
            #   以前要求到得了最深点(接触点 + 5cm),当前构型差 2cm 就去换构型分支,甩了 154°(2026-11-14 真机)
            if self._final_segment_ok(arm, plan.path[-1], contact, cand["R"], blocked, contact, d):
                first = {"ok": True, "kind": "arm", "plan": plan, "summary": plan.summary()}
                if target is not contact and not self._final_segment_ok(arm, plan.path[-1], target, cand["R"],
                                                                        blocked, contact, d):
                    first["shallow"] = True          # 这个构型越过接触点压不到最深点(plan_touch 会先试多前倾)
                return first, None
            why = "末端规划到得了预接触点,但到的那个构型直线下不去"
        # 关节空间直接去验过的关节角:换构型分支时手臂会大幅甩动,贴着桌面很危险。
        #   2026-11-14 真机:末端只要往下 7cm,却走了关节最大变化 154° 的 RRT,执行完 joint3 差 30.8° 没到位
        change = float(np.degrees(np.max(np.abs(np.asarray(cand["q"]) - self.arm.q_arm(arm)))))
        limit = self.touch["max_joint_fallback_deg"]
        if change > limit:
            return None, f"{why};改关节空间要转 {change:.0f}° > {limit}°,不走"
        names = self.arm.kins[arm].joint_names
        goal = {n: float(v) for n, v in zip(names, cand["q"])}
        joints = self.plan_joints(goal, self.touch["joint_speed_rad_s"])
        if not joints["ok"]:
            return None, f"{why};关节空间 {joints['error']}"
        joints["summary"] = f"{why},改关节空间:{joints['summary']}"
        return joints, None

    def shoulder_xyz(self, arm):
        """肩心(link2 原点)在 base_link 的位置,用来判断物体在不在手臂够得着的范围。"""
        pb, Rb = self.arm._root_in_base()
        kin = self.arm.solvers[arm].kin
        return (pb + Rb @ kin.segment_positions(np.zeros(kin.n))[1]).tolist()

    def _goal_rotation(self, arm, target, how, rpy_deg, roll_deg=0.0, tool_axis=None):
        """终点姿态(base_link 的 3x3)。keep 且不转手腕时返回 None(= 规划器取当前姿态)。算不出返回 (None, 原因)。"""
        if how == "exact":
            return rpy_deg_to_matrix(rpy_deg), ""
        if how == "axis":
            R_now = self.arm.pose(arm)[1]
            return roll_locked(R_now, tool_axis, roll_deg), ""
        if how == "keep":
            if not roll_deg:
                return None, ""
            R_now = self.arm.pose(arm)[1]
            return roll_locked(R_now, R_now[:, 2], roll_deg), f"(手腕转 {roll_deg:.0f}°)"
        pb, Rb = self.arm._root_in_base()
        T_root = Rb.T @ (target - pb)
        q0 = self.arm.q_arm(arm)
        solver = self.arm.solvers[arm]

        def valid(q):
            return self.arm.valid(arm, q)

        if how == "free":
            q, note = free_goal(solver, q0, T_root, valid, roll_deg)
        elif how == "comfort":
            if arm not in self._comfort:
                self._comfort[arm] = ComfortPose(self.arm.urdf, arm, solver)
            left = Rb.T @ np.array([0.0, 1.0, 0.0])
            if arm == "left":
                outward = left
            else:
                outward = -left
            best, tried = self._comfort[arm].solve(T_root, q0, Rb.T @ np.array([0.0, 0.0, 1.0]), outward, valid,
                                                   roll_deg=roll_deg)
            if best is None:
                return None, f"{tried} 个舒适候选都解不出(或都会撞)"
            q, note = best["q"], f"(基面 {best['phi_deg']}°,舒适分 {best['score']})"
        else:
            return None, f"不认识的姿态策略 {how}"
        if q is None:
            return None, note
        return Rb @ solver.kin.fk(q)[1], ""

    def _lowest_z(self, arm, path):
        """整条路径上末端的最低高度(base_link),用于防止路径穿过桌面。"""
        p_root, R_root = self.arm._root_in_base()
        kin = self.arm.kins[arm]
        return min(float((p_root + R_root @ kin.fk(np.asarray(q))[0])[2]) for q in path)

    def plan_joints(self, target, speed):
        """关节组到 target({关节名: 弧度}):整机碰撞检查,直线会撞就经录好位姿中转 / 关节 RRT。"""
        detour = JointDetour(self.body._valid, self.joint_limits(), self.poses)
        plan = self.body.plan(target, speed=speed, detour=detour)
        if isinstance(plan, str):
            return {"ok": False, "error": plan}
        return {"ok": True, "kind": "joints", "plan": plan, "summary": plan.summary()}

    # ---------------- 执行 ----------------
    def execute(self, plan):
        kind = plan["kind"]
        if kind == "arm":
            ok, msg = self.arm.execute(plan["plan"])
        elif kind == "joints":
            ok, msg = self.body.run(plan["plan"])
        elif kind == "touch":
            return self._execute_touch(plan)
        elif kind == "body_then":
            return self._execute_body_then(plan)
        else:
            return {"ok": False, "msg": f"不认识的计划类型 {kind}"}
        return {"ok": ok, "msg": msg}

    def _execute_body_then(self, plan):
        """先动腰;到位后从新姿态重新规划手臂(这次不再动腰),再执行。"""
        ok, msg = self.body.run(plan["body"]["plan"])
        if not ok:
            return {"ok": False, "msg": f"动腰失败:{msg}"}
        moved = f"腰已经前倾 {plan['lean_deg']}°、转 {plan['turn_deg']}°(机器人动了)"
        after = plan["after"]
        try:
            if after["kind"] == "touch":
                arm_plan = self.plan_touch(**after["args"], allow_body=False)
            else:
                arm_plan = self.plan_arm_to(**after["args"], allow_body=False)
            if not arm_plan["ok"]:
                return {"ok": False, "msg": f"{moved},但手臂规划失败:{arm_plan['error']}"}
            done = self.execute(arm_plan)
        except Exception as e:                     # 腰已经动了:报错也要让模型知道这一点
            return {"ok": False, "msg": f"{moved},之后手臂出错:{type(e).__name__}: {e}"}
        result = {"ok": done["ok"], "msg": f"腰:{msg};手臂({arm_plan['summary']}):{done['msg']}"}
        if "contact" in done:
            result["contact"] = done["contact"]
        return result

    def _execute_touch(self, plan):
        """先走第一段(查碰撞)到预接触点;到了以后从实际位姿、保持实际姿态,慢速直线到接触点(只放过物体附近的点云)。

        有六维力时(touch.force,2026-11-14 加):
          · 到预接触点后静止取零点;最后一段边走边看力,比零点变了 threshold_n 以上 = 碰到东西(桌面 / 物体),马上停;
          · gap 为 0(“碰到”“抓取下探”)时终点再往前 overshoot_m:物体坐标只是可见表面的中位点,
            真正的“底”以力为准,比如张开的夹爪罩住鼠标往下,碰到桌面才停;
          · 碰到后沿接近方向往回退 retreat_m(给闭合夹爪留空间),这次 touch 就算完成。
        收不到力:退回原来的做法(只走到接触点,不多走),结果里写明。"""
        arm = plan["arm"]
        cfg = self.touch
        first_msg = "已在跟前"
        if plan["first"] is not None:
            done = self.execute(plan["first"])
            if not done["ok"]:
                return {"ok": False, "msg": f"第一段(查碰撞,到预接触点)失败:{done['msg']}"}
            first_msg = done["msg"]
        contact = np.array(plan["contact_xyz_m"])
        d = np.asarray(plan["direction"], dtype=float)
        now = np.array(self.end_pose(arm)["xyz_m"])
        left = float(np.linalg.norm(contact - now))
        if left > cfg["backoffs_cm"][-1] / 100 + 0.02:
            return {"ok": False, "msg": f"到预接触点后离接触点还有 {left:.2f}m,超过最后一段的上限,停下"}

        force_cfg = cfg["force"]
        frame = force_cfg["frames"][arm]
        zero, force_note = self._force_zero(frame)
        expect_contact = plan["gap_m"] == 0          # gap 0 = 要碰上;gap > 0 = 停在跟前,碰到力只是保护
        extras = [0.0]                                # 接触点之后再多走多少,从多到少试(手臂快伸直时多走的够不着)
        if zero is not None and expect_contact:
            extras = []
            extra = force_cfg["overshoot_m"]
            while extra > 0.0:                         # 5、4、3、2、1cm、0:这个构型能多走多少就多走多少
                extras.append(round(extra, 3))
                extra -= 0.01
            extras.append(0.0)
        try:
            R_now = self.arm.pose(arm)[1]
            for extra in extras:
                last = self.arm.plan_cartesian(arm, contact + extra * d, R_now, ignore_cloud=True)
                if not isinstance(last, str):
                    break
            if isinstance(last, str):
                return {"ok": False, "msg": f"已到预接触点;最后一段直线算不出来:{last}"}
            blocked = outside_object(self.arm.octomap_base(), plan["object_xyz_m"],
                                     cfg["keep_out"]["object_radius_m"])
            if not self._keep_out_ok(arm, last.path, blocked, contact, d):   # 用最新点云再查一次:规划后桌上多了东西也拦得住
                return {"ok": False, "msg": "已到预接触点,停下:最后一段夹爪会碰到物体以外的点云(桌面或别的东西)"}
            slow_down(last, cfg["final_speed_m_s"])
            if zero is None:
                ok, msg = self.arm.execute(last)
                if not ok:
                    return {"ok": False, "msg": f"已到预接触点;最后一段失败:{msg}"}
                return {"ok": True, "msg": f"{first_msg};最后 {left * 100:.0f}cm 直线到达接触点:{msg}({force_note})",
                        "contact": {"felt": None, "note": force_note}}
            done = self._run_force_guarded(arm, last, frame, zero, d, first_msg, expect_contact)
            done.setdefault("contact", {})["overshoot_m"] = round(extra, 3)   # 这次最多能越过接触点多远
            return done
        finally:
            if self.wrench is not None:
                self.wrench.stop()

    def _force_zero(self, frame):
        """开始订六维力并在静止时取零点。返回 (零点, 说明);用不了时零点为 None。"""
        force_cfg = self.touch["force"]
        if self.wrench is None:
            return None, "没开六维力(touch.force.enabled),最后一段只走到接触点"
        if not self.wrench.start(1.0):
            return None, f"1s 没收到六维力({frame}),最后一段只走到接触点,不多走"
        try:
            zero = self.wrench.average(frame, force_cfg["tare_s"], force_cfg["max_age_s"])
        except RuntimeError as e:
            return None, f"{e},最后一段只走到接触点,不多走"
        return zero, "六维力零点已取"

    def _run_force_guarded(self, arm, last, frame, zero, d, first_msg, expect_contact):
        """边执行最后一段边看力:变化超过阈值就取消轨迹,停稳后沿接近方向往回退 retreat_m。
        expect_contact=False(gap > 0,停在跟前):没碰到是正常的,碰到了(比如对方的手迎上来)同样停 + 退。"""
        force_cfg = self.touch["force"]
        start = np.array(self.end_pose(arm)["xyz_m"])
        outcome = {}

        def run_last():
            outcome["result"] = self.arm.execute(last)

        worker = threading.Thread(target=run_last, daemon=True)
        worker.start()
        hit = None
        change = 0.0
        peak = 0.0
        while worker.is_alive():
            try:
                force = self.wrench.latest(frame, force_cfg["max_age_s"])
            except RuntimeError as e:                    # 力反馈断了:立刻停
                self.arm.cancel()
                worker.join(5.0)
                return {"ok": False, "msg": f"已到预接触点;最后一段走到一半{e},已停"}
            change = force_change_n(force, zero)
            peak = max(peak, change)
            if change > force_cfg["threshold_n"]:
                self.arm.cancel()
                hit = force
                break
            time.sleep(0.002)
        worker.join(last.duration + 10.0)
        moved = float(np.linalg.norm(np.array(self.end_pose(arm)["xyz_m"]) - start))
        if hit is None:
            ok, msg = outcome.get("result", (False, "执行线程没返回"))
            if not ok:
                return {"ok": False, "msg": f"已到预接触点;最后一段失败:{msg}"}
            if not expect_contact:
                return {"ok": True, "msg": f"{first_msg};最后 {moved * 100:.1f}cm 直线到达,停在物体跟前:{msg}",
                        "contact": {"felt": False, "moved_m": round(moved, 3)}}
            return {"ok": True, "msg": f"{first_msg};最后一段走完 {moved * 100:.1f}cm 也没感到接触力"
                                      f"(力变化最大 {peak:.1f}N < 阈值 {force_cfg['threshold_n']}N),可能没碰到",
                    "contact": {"felt": False, "moved_m": round(moved, 3)}}

        time.sleep(force_cfg["settle_s"])
        felt = {"felt": True, "force_change_n": round(change, 1), "moved_m": round(moved, 3),
                "force_n": [round(float(v), 1) for v in hit[:3]]}
        now, R_now = self.arm.pose(arm)
        back = self.arm.plan_cartesian(arm, now - force_cfg["retreat_m"] * d, R_now, ignore_cloud=True)
        if isinstance(back, str):
            return {"ok": False, "msg": f"走了 {moved * 100:.1f}cm 碰到东西(力变化 {change:.1f}N)已停,"
                                        f"但往回退 {force_cfg['retreat_m'] * 100:.0f}cm 算不出来:{back}",
                    "contact": felt}
        slow_down(back, self.touch["final_speed_m_s"])
        ok, msg = self.arm.execute(back)
        if not ok:
            return {"ok": False, "msg": f"碰到东西已停,往回退失败:{msg}", "contact": felt}
        felt["retreat_m"] = force_cfg["retreat_m"]
        return {"ok": True, "msg": f"{first_msg};最后一段走了 {moved * 100:.1f}cm 碰到东西(力变化 {change:.1f}N),"
                                  f"已停并往回退 {force_cfg['retreat_m'] * 100:.0f}cm",
                "contact": felt}

    def close(self):
        self.arm.cancel()
        self.body.cancel()
        self.camera.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.arm.destroy_node()
        self.node.destroy_node()
        rclpy.try_shutdown()
