"""真机器人:手臂运控(harness/motion/)+ 头部 RGBD(harness/perception/),都是本项目里的代码。

  · 手臂末端:ArmMotion —— 先积分笛卡尔直线,会撞再 plan_detour(via_then_rrt,和 pose_ab_search 同一套)
  · 关节组(腰/头/双臂):WholeBody —— 关节直线插值 + 整机碰撞检查,会撞再 JointDetour(经录好位姿中转 / 关节 RRT)
  · touch_object:plan_touch —— 唯一可以不查点云碰撞的运动,固定两段(查碰撞到预接触点 → 不查点云慢速直线贴上去)
  · 夹爪:GdkGripper(GDK 伺服话题)

运行前要 source ~/agi@g2_robot/env_g2.sh(GDK 的 DDS 配置),并且 hardware.launch.py 已经在跑。
"""
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
from .gripper_gdk import GdkGripper


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
             P 处不撞(含点云),整段除了点云不撞(自碰撞照查)
          ② 验过的 P 才规划第一段(全程查碰撞):直线 → 末端绕障 via_then_rrt → 关节空间直接去 P 的关节角
             (经录好位姿中转 / RRT)。终点构型已经验证过不撞,只要连得通,关节空间就找得到 ——“能去就一定规划得出来”
          ③ 执行时到了 P,再从实际位姿不查点云、慢速直线到 C(_execute_touch)
        接近方向依次试:末端 → 物体(direction)、肩 → 物体水平、从上往下。
        哪个 P 都没有 IK 解(手臂够不着):allow_body 时先动腰,到位后整个重来。"""
        cfg = self.touch
        obj = np.asarray(object_xyz, dtype=float)
        start = np.array(self.end_pose(arm)["xyz_m"])
        d0 = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
        contact0 = obj - gap_m * d0
        near = float(np.linalg.norm(contact0 - start))
        if near <= cfg["backoffs_cm"][0] / 100:      # 已经在跟前:只走最后一段
            return {"ok": True, "kind": "touch", "arm": arm, "first": None, "contact_xyz_m": contact0.tolist(),
                    "precontact_xyz_m": start.tolist(), "unchecked_m": round(near, 3), "approach": "末端→物体",
                    "summary": f"已经在接触点 {near * 100:.0f}cm 内,直接不查点云直线过去"}

        counts = {"contact": 0, "ik": 0, "collide": 0, "last": 0}
        failures = []
        tried = 0
        for name, d in self._touch_directions(arm, obj, d0):
            contact = obj - gap_m * d
            rolls = []                                    # [(roll, 接触点姿态 + 关节角)],够不着接触点的 roll 不要
            for roll in cfg["roll_offsets_deg"]:
                contact_pose = self._touch_contact_pose(arm, contact, d, roll)
                if contact_pose is None:
                    counts["contact"] += 1
                else:
                    rolls.append((roll, contact_pose))
            for b_cm in cfg["backoffs_cm"]:
                pre = contact - b_cm / 100 * d
                for roll, contact_pose in rolls:
                    cand = self._touch_candidate(arm, pre, contact_pose)
                    if isinstance(cand, str):
                        counts[cand] += 1
                        continue
                    tried += 1
                    first, why = self._reach_precontact(arm, pre, cand, contact)
                    if first is not None:
                        return {"ok": True, "kind": "touch", "arm": arm, "first": first,
                                "contact_xyz_m": contact.tolist(), "precontact_xyz_m": pre.tolist(),
                                "unchecked_m": b_cm / 100, "approach": name, "roll_offset_deg": roll,
                                "summary": f"接近方向 {name};① 查碰撞到接触点前 {b_cm}cm:{first['summary']};"
                                           f"② 不查点云直线 {b_cm}cm 到接触点({cfg['final_speed_m_s']} m/s)"}
                    failures.append(f"{name} {b_cm}cm:第一段规划不过去({why})")
                    if tried >= cfg["max_plan_tries"]:
                        return {"ok": False, "error": " | ".join(failures) + f" | 已试 {tried} 个预接触点,停止"}
                    break                    # 这个 b 规划不通,换下一个 b(别的 roll 多半一样)
        why = (f"接触点按要求的朝向够不着 {counts['contact']} 次;预接触点 {counts['ik']} 个 IK 无解、"
               f"{counts['collide']} 个 IK 解都会撞、{counts['last']} 个最后一段走不通或会自碰撞")
        failures.append(why)
        if tried == 0 and counts["collide"] == 0 and counts["last"] == 0 and allow_body \
                and self.body_assist and self.body_assist.get("enabled"):
            pre = contact0 - cfg["backoffs_cm"][0] / 100 * d0
            after = {"kind": "touch", "args": {"arm": arm, "object_xyz": obj.tolist(), "gap_m": gap_m,
                                               "direction": d0.tolist()}}
            body = self._body_assist(arm, pre, d0.tolist(), 0.0, failures, after)
            if body is not None:
                return body
        return {"ok": False, "error": " | ".join(failures)}

    def _touch_directions(self, arm, obj, direction):
        """接近方向候选(单位向量,从手指向物体),彼此差 15° 以上才算不同。"""
        cands = [("末端→物体", direction)]
        horizontal = obj - np.array(self.shoulder_xyz(arm))
        horizontal[2] = 0.0
        if np.linalg.norm(horizontal) > 1e-6:
            cands.append(("肩→物体水平", horizontal))
        cands.append(("从上往下", np.array([0.0, 0.0, -1.0])))
        result = []
        for name, d in cands:
            d = np.asarray(d, dtype=float) / np.linalg.norm(d)
            if all(_angle_deg(d, e) > 15.0 for _, e in result):
                result.append((name, d))
        return result

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

    def _touch_candidate(self, arm, pre, contact_pose):
        """离线验一个预接触点:从接触点的关节角沿直线反着走到 P(同一个构型分支,保证 P → C 直线走得通),
        P 处不撞(含点云),整段除了点云不撞。返回 {"q", "R"} 或失败种类 "ik" / "collide" / "last"。"""
        pb, Rb = self.arm._root_in_base()
        solver = self.arm.solvers[arm]
        R_root = Rb.T @ contact_pose["R"]
        path, reached, _, _, _ = solver.solve(contact_pose["q"], Rb.T @ (pre - pb), "full", R_root, R_root[:, 0])
        if not reached:
            return "ik"
        q_pre = np.asarray(path[-1], dtype=float)
        if not self.arm.valid(arm, q_pre):
            return "collide"
        if not self.arm.path_clear(arm, path[::-1], ignore_cloud=True):
            return "last"
        return {"q": q_pre, "R": contact_pose["R"]}

    def _final_segment_ok(self, arm, q, contact, R):
        """从关节角 q、保持姿态 R,积分笛卡尔直线能不能到接触点,且除了点云不撞。"""
        pb, Rb = self.arm._root_in_base()
        solver = self.arm.solvers[arm]
        R_root = Rb.T @ R
        path, reached, _, _, _ = solver.solve(np.asarray(q, dtype=float), Rb.T @ (contact - pb), "full",
                                              R_root, R_root[:, 0])
        return reached and self.arm.path_clear(arm, path, ignore_cloud=True)

    def _reach_precontact(self, arm, pre, cand, contact):
        """第一段(全程查碰撞,含点云)到预接触点:直线 → 末端绕障 → 关节空间(经录好位姿中转 / RRT)。
        前两种按末端位姿规划,到了以后的构型分支可能和验过的不一样:要再验一次“从这里直线下得去”,
        不行就改关节空间直接去验过的那个关节角。返回 (计划, None) 或 (None, 原因)。"""
        plan, why = self._line_or_detour(arm, pre, cand["R"])
        if plan is not None:
            if self._final_segment_ok(arm, plan.path[-1], contact, cand["R"]):
                return {"ok": True, "kind": "arm", "plan": plan, "summary": plan.summary()}, None
            why = "末端规划到得了预接触点,但到的那个构型直线下不去"
        names = self.arm.kins[arm].joint_names
        target = {n: float(v) for n, v in zip(names, cand["q"])}
        joints = self.plan_joints(target, self.touch["joint_speed_rad_s"])
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
        return {"ok": done["ok"], "msg": f"腰:{msg};手臂({arm_plan['summary']}):{done['msg']}"}

    def _execute_touch(self, plan):
        """先走第一段(查碰撞)到预接触点;到了以后从实际位姿、保持实际姿态,不查点云慢速直线到接触点。"""
        arm = plan["arm"]
        cfg = self.touch
        first_msg = "已在跟前"
        if plan["first"] is not None:
            done = self.execute(plan["first"])
            if not done["ok"]:
                return {"ok": False, "msg": f"第一段(查碰撞,到预接触点)失败:{done['msg']}"}
            first_msg = done["msg"]
        target = np.array(plan["contact_xyz_m"])
        now = np.array(self.end_pose(arm)["xyz_m"])
        left = float(np.linalg.norm(target - now))
        if left > cfg["backoffs_cm"][-1] / 100 + 0.02:
            return {"ok": False, "msg": f"到预接触点后离接触点还有 {left:.2f}m,超过不查点云的上限,停下"}
        last = self.arm.plan_cartesian(arm, target, self.arm.pose(arm)[1], ignore_cloud=True)
        if isinstance(last, str):
            return {"ok": False, "msg": f"已到预接触点;最后一段直线算不出来:{last}"}
        slow_down(last, cfg["final_speed_m_s"])
        ok, msg = self.arm.execute(last)
        if not ok:
            return {"ok": False, "msg": f"已到预接触点;最后一段失败:{msg}"}
        return {"ok": True, "msg": f"{first_msg};最后 {left * 100:.0f}cm 不查点云直线到达:{msg}"}

    def close(self):
        self.arm.cancel()
        self.body.cancel()
        self.camera.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.arm.destroy_node()
        self.node.destroy_node()
        rclpy.try_shutdown()
