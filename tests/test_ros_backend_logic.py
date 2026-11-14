"""RosBackend 的规划逻辑(plan_arm_to 的步骤列表、plan_touch 的固定链路、动腰)不连 ROS:
手臂 / 腰换成替身,IK 用真实 URDF。“点云”= 鼠标周围 obstacle_radius_m 的球:夹爪尖进了球就算撞。

    source /opt/ros/humble/setup.bash
    python3 -m pytest tests/test_ros_backend_logic.py -q
"""
import math
import os

import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyKDL")

from harness.backends.ros import RosBackend  # noqa: E402
from harness.motion.algorithms import ArmSolver  # noqa: E402
from harness.motion.kin import arm_kin, body_kin  # noqa: E402

URDF_PATH = os.environ.get("G2_URDF", os.path.expanduser(
    "~/agi@g2_robot/src/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_ee.urdf"))   # 真机用的,有 TCP
MOUSE = np.array([0.7931, -0.1413, 0.7195])                 # 真机 2026-11-13 01:55 的鼠标
BODY0 = np.radians([-39.85, 89.9, -49.62, 0, 0])
RIGHT0 = np.radians([-112.2, 36.57, 85.82, -99.41, 127.79, 12.84, 21.11])
TOUCH = {"default_gap_cm": 2, "max_gap_cm": 10, "backoffs_cm": [5, 8, 12, 16, 20, 25, 30],
         "roll_offsets_deg": [0, 45, -45, 90, -90], "max_plan_tries": 4, "final_speed_m_s": 0.03,
         "joint_speed_rad_s": 0.2}


class Traj:
    points = []


class P:
    def __init__(self, path):
        self.path = path
        self.traj = Traj()
        self.duration = 1.0

    def summary(self):
        return f"{len(self.path)} 点"


def make_backend(obstacle_radius_m, line_works=True):
    """line_works=False:直线和末端绕障都规划不通(只剩关节空间)。"""
    if not os.path.exists(URDF_PATH):
        pytest.skip(f"没有 URDF {URDF_PATH}")
    from urdf_parser_py.urdf import URDF
    urdf = open(URDF_PATH, encoding="utf-8").read()
    bk = body_kin(urdf)
    solver = ArmSolver(arm_kin(urdf, "right"))
    limits = {j.name: (j.limit.lower, j.limit.upper) for j in URDF.from_xml_string(urdf).joints if j.limit is not None}
    state = {"qb": BODY0.copy(), "qa": RIGHT0.copy(), "body_moves": 0, "ignore_cloud_calls": 0, "joint_plans": []}

    def tip(q):
        pb, Rb = bk.fk(state["qb"])
        return pb + Rb @ solver.kin.fk(np.asarray(q))[0]

    def hits_cloud(q):
        return np.linalg.norm(tip(q) - MOUSE) < obstacle_radius_m

    class Arm:
        solvers = {"right": solver}
        kins = {"right": solver.kin}
        bkin = bk

        def _root_in_base(self):
            return bk.fk(state["qb"])

        def q_arm(self, arm):
            return state["qa"]

        def pose(self, arm):
            pb, Rb = bk.fk(state["qb"])
            p, R = solver.kin.fk(state["qa"])
            return pb + Rb @ p, Rb @ R

        def valid(self, arm, q, ignore_cloud=False):
            return ignore_cloud or not hits_cloud(q)

        def path_clear(self, arm, path, ignore_cloud=False):
            if ignore_cloud:
                state["ignore_cloud_calls"] += 1
            return all(self.valid(arm, q, ignore_cloud) for q in path)

        def plan_cartesian(self, arm, p, R=None, mode="full", ignore_cloud=False):
            if ignore_cloud:
                state["ignore_cloud_calls"] += 1
            elif not line_works:
                return "直线会撞(替身)"
            pb, Rb = bk.fk(state["qb"])
            R = self.pose(arm)[1] if R is None else R
            Rr = Rb.T @ R
            path, ok, ep, _, _ = solver.solve(state["qa"], Rb.T @ (np.asarray(p) - pb), "full", Rr, Rr[:, 0])
            if not ok:
                return f"没到位 {ep * 1000:.0f}mm"
            if not self.path_clear(arm, path, ignore_cloud):
                return "会撞(鼠标在点云里)"
            return P(path)

        def plan_detour(self, arm, p, R=None):
            return "[via_then_rrt] 会撞"

        def execute(self, plan):
            state["qa"] = np.asarray(plan.path[-1])
            return True, "JTC 成功"

    class Body:
        def run(self, plan):
            if isinstance(plan, dict) and "arm" in plan:          # 关节空间走手臂(替身计划)
                state["qa"] = np.asarray(plan["arm"])
            else:
                state["qb"] = np.array([plan[n] for n in bk.joint_names])
                state["body_moves"] += 1
            return True, "关节到位"

    def plan_joints(target, speed):
        names = list(target)
        if names == list(solver.kin.joint_names):
            state["joint_plans"].append(target)
            return {"ok": True, "kind": "joints", "plan": {"arm": [target[n] for n in names]}, "summary": "手臂关节 RRT"}
        return {"ok": True, "kind": "joints", "plan": target, "summary": "腰关节直线"}

    b = RosBackend.__new__(RosBackend)
    b.arm, b.body, b.speed, b._comfort, b._limits = Arm(), Body(), 0.05, {}, limits
    b.tool_axis_tolerance_deg = 15.0
    b.touch = TOUCH
    b.body_assist = {"enabled": True, "max_lean_deg": 30, "max_turn_deg": 20, "speed_rad_s": 0.1}
    b._lowest_z = lambda arm, path: 0.0
    b.joint_state = lambda: {n: float(v) for n, v in zip(bk.joint_names, state["qb"])}
    b.plan_joints = plan_joints
    b.end_pose = lambda arm: {"xyz_m": b.arm.pose(arm)[0].tolist(), "rpy_deg": [0, 0, 0], "link": "tcp_r_link"}
    return b, state


def lean_above_mouse(b, state):
    above = b.plan_arm_to("right", MOUSE + [0, 0, 0.25], orientation=("comfort", "keep", "free"), tool_axis=[0, 0, -1.0])
    assert above["ok"] and above["kind"] == "body_then", above.get("error")
    assert b.execute(above)["ok"] and state["body_moves"] == 1


def test_mouse_above_then_touch_from_above():
    """“低头找鼠标,拿起来”:上方 25cm(要动腰)→ touch 下到鼠标上表面(鼠标在点云里,只能走 touch 链路)。"""
    b, state = make_backend(obstacle_radius_m=0.10)
    lean_above_mouse(b, state)
    touch = b.plan_touch("right", MOUSE, 0.0, [0, 0, -1.0])
    assert touch["ok"] and touch["kind"] == "touch", touch.get("error")
    assert touch["unchecked_m"] >= 0.10                      # 预接触点在“点云”外
    assert np.linalg.norm(np.array(touch["precontact_xyz_m"]) - MOUSE) >= 0.10
    done = b.execute(touch)
    assert done["ok"], done["msg"]
    end, R = b.arm.pose("right")
    assert np.linalg.norm(end - MOUSE) < 0.01
    assert math.degrees(math.acos(-R[2, 2])) < 16             # 夹爪基本朝下(严格朝下够不着,容差 15°)


def test_touch_first_segment_falls_back_to_joint_space():
    """直线、末端绕障都不通时,第一段改走关节空间(终点构型已验证不撞)——能去就一定规划得出来。"""
    b, state = make_backend(obstacle_radius_m=0.10)
    lean_above_mouse(b, state)
    b2, state2 = make_backend(obstacle_radius_m=0.10, line_works=False)
    state2["qb"], state2["qa"] = state["qb"].copy(), state["qa"].copy()
    touch = b2.plan_touch("right", MOUSE, 0.0, [0, 0, -1.0])
    assert touch["ok"], touch.get("error")
    assert touch["first"]["kind"] == "joints" and len(state2["joint_plans"]) == 1
    assert b2.execute(touch)["ok"]
    assert np.linalg.norm(b2.arm.pose("right")[0] - MOUSE) < 0.01


def test_plan_arm_to_never_skips_cloud():
    """approach_object / move_arm_end 走的 plan_arm_to:目标在点云里就是失败,不会不查点云贴过去。"""
    b, state = make_backend(obstacle_radius_m=0.10)
    lean_above_mouse(b, state)
    result = b.plan_arm_to("right", MOUSE + [0, 0, 0.02], orientation=("keep",), tool_axis=[0, 0, -1.0],
                           allow_body=False)
    assert not result["ok"]
    assert state["ignore_cloud_calls"] == 0


def test_touch_body_assist_replans_touch_without_moving_body_again():
    """够不着时 touch 先动腰;腰到位后重新 plan_touch,而且不再动腰(以前嵌套出过 KeyError: 'plan')。"""
    b, state = make_backend(obstacle_radius_m=0.10)
    far = MOUSE + [0.15, 0, 0]                               # 比鼠标再远 15cm:现在的腰姿态够不着
    plan = b.plan_touch("right", far, 0.0, [0, 0, -1.0])
    assert plan["ok"] and plan["kind"] == "body_then", plan.get("error")
    assert plan["after"]["kind"] == "touch"
    calls = []
    original = b.plan_touch

    def spy(*a, **k):
        calls.append(k.get("allow_body", True))
        return original(*a, **k)

    b.plan_touch = spy
    done = b.execute(plan)
    assert done["ok"], done["msg"]
    assert calls == [False] and state["body_moves"] == 1
    assert np.linalg.norm(b.arm.pose("right")[0] - far) < 0.01
