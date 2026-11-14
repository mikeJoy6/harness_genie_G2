"""RosBackend 的规划逻辑(plan_arm_to 的步骤列表、plan_touch 的固定链路、动腰)不连 ROS:
手臂 / 腰换成替身,IK 用真实 URDF。“点云”= 鼠标周围 obstacle_radius_m 的球:夹爪尖进了球就算撞。

    source /opt/ros/humble/setup.bash
    python3 -m pytest tests/test_ros_backend_logic.py -q
"""
import math
import os
import time

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
         "joint_speed_rad_s": 0.2, "max_joint_fallback_deg": 90,
         "keep_out": {"object_radius_m": 0.08, "gripper_radius_m": 0.04, "gripper_length_m": 0.13},
         "force": {"enabled": True, "frames": {"left": "arm_l_end_link", "right": "arm_r_end_link"},
                   "threshold_n": 5.0, "tare_s": 0.02, "max_age_s": 0.1, "overshoot_m": 0.05, "retreat_m": 0.02,
                   "settle_s": 0.0}}


class Traj:
    points = []


class P:
    def __init__(self, path):
        self.path = path
        self.traj = Traj()
        self.duration = 1.0

    def summary(self):
        return f"{len(self.path)} 点"


def make_backend(obstacle_radius_m, line_works=True, cloud=None, wrench=None, table_z=None):
    """line_works=False:直线和末端绕障都规划不通(只剩关节空间)。
    cloud:octomap_base() 返回的点云格(base_link),给 touch 最后一段的几何检查用;默认没有。
    wrench:None = 没有六维力;"ok" = 夹爪尖低于 table_z 时力 20N;"silent" = 收不到;"lost" = 走着走着断了。"""
    if not os.path.exists(URDF_PATH):
        pytest.skip(f"没有 URDF {URDF_PATH}")
    from urdf_parser_py.urdf import URDF
    urdf = open(URDF_PATH, encoding="utf-8").read()
    bk = body_kin(urdf)
    solver = ArmSolver(arm_kin(urdf, "right"))
    limits = {j.name: (j.limit.lower, j.limit.upper) for j in URDF.from_xml_string(urdf).joints if j.limit is not None}
    state = {"qb": BODY0.copy(), "qa": RIGHT0.copy(), "body_moves": 0, "ignore_cloud_calls": 0, "joint_plans": [],
             "cancel": False, "running": False, "force_reads": 0}

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

        def octomap_base(self):
            return np.empty((0, 3)) if cloud is None else np.asarray(cloud)

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
            """一个点一个点走(像真 JTC 一样要花时间),别的线程 cancel() 就停在当前点。"""
            state["cancel"] = False
            state["running"] = True
            try:
                for q in plan.path:
                    if state["cancel"]:
                        return False, "JTC 已取消"
                    state["qa"] = np.asarray(q)
                    time.sleep(0.004)
            finally:
                state["running"] = False
            return True, "JTC 成功"

        def cancel(self):
            state["cancel"] = True
            return True

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

    class Wrench:
        def start(self, wait_s=1.0):
            return wrench != "silent"

        def stop(self):
            pass

        def average(self, frame, duration_s, max_age_s=0.1):
            return np.zeros(6)

        def latest(self, frame, max_age_s=0.1):
            state["force_reads"] += 1
            if wrench == "lost" and state["force_reads"] > 3:
                raise RuntimeError(f"六维力 {frame} {max_age_s}s 没更新(替身)")
            if table_z is not None and tip(state["qa"])[2] < table_z:
                return np.array([0.0, 0.0, 20.0, 0.0, 0.0, 0.0])         # 桌面从下面顶上来
            return np.zeros(6)

    b = RosBackend.__new__(RosBackend)
    b.wrench = None if wrench is None else Wrench()
    b.arm, b.body, b.speed, b._comfort, b._limits = Arm(), Body(), 0.05, {}, limits
    b.tool_axis_tolerance_deg = 15.0
    b.touch = TOUCH
    b.body_assist = {"enabled": True, "max_lean_deg": 30, "max_turn_deg": 20, "speed_rad_s": 0.1}
    b._lowest_z = lambda arm, path: 0.0
    b.joint_state = lambda: {n: float(v) for n, v in zip(bk.joint_names, state["qb"])}
    b.plan_joints = plan_joints
    b.end_pose = lambda arm: {"xyz_m": b.arm.pose(arm)[0].tolist(), "rpy_deg": [0, 0, 0], "link": "tcp_r_link"}
    return b, state


def plan_and_run_touch(b, state, mouse, gap_m=0.0):
    """plan_touch 可能先要动腰(body_then:换构型要大幅甩臂,或压不到足够深时先多前倾),都执行完。
    全程不许走关节空间大幅甩臂(2026-11-14 真机甩了 154°、没到位)。"""
    plan = b.plan_touch("right", mouse, gap_m, [0, 0, -1.0])
    assert plan["ok"] and plan["kind"] in ("touch", "body_then"), plan.get("error")
    done = b.execute(plan)
    assert state["joint_plans"] == [], "不该走关节空间"
    return plan, done


def lean_above_mouse(b, state):
    above = b.plan_arm_to("right", MOUSE + [0, 0, 0.25], orientation=("comfort", "keep", "free"), tool_axis=[0, 0, -1.0])
    assert above["ok"] and above["kind"] == "body_then", above.get("error")
    assert b.execute(above)["ok"] and state["body_moves"] == 1


def test_mouse_above_then_touch_from_above():
    """“低头找鼠标,拿起来”:上方 25cm(要动腰)→ touch 下到鼠标上表面(鼠标在点云里,只能走 touch 链路)。"""
    b, state = make_backend(obstacle_radius_m=0.10)
    lean_above_mouse(b, state)
    touch, done = plan_and_run_touch(b, state, MOUSE)
    assert done["ok"], done["msg"]
    end, R = b.arm.pose("right")
    assert np.linalg.norm(end - MOUSE) < 0.01
    assert math.degrees(math.acos(-R[2, 2])) < 16             # 夹爪基本朝下(严格朝下够不着,容差 15°)


def test_touch_first_segment_falls_back_to_joint_space():
    """直线、末端绕障都不通时,第一段改走关节空间(终点构型已验证不撞)——能去就一定规划得出来。"""
    b, state = make_backend(obstacle_radius_m=0.10)
    lean_above_mouse(b, state)
    b2, state2 = make_backend(obstacle_radius_m=0.10, line_works=False)
    b2.touch = dict(TOUCH, max_joint_fallback_deg=360)       # 这里只测“关节空间兜底”这个机制本身
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


def table_scene(mouse):
    """桌面(3cm 一格,中心比鼠标顶面低 2.5cm)+ 鼠标那几格。"""
    xs = np.arange(0.45, 1.10, 0.03)
    ys = np.arange(-0.60, 0.20, 0.03)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    table = np.stack([X.ravel(), Y.ravel(), np.full(X.size, mouse[2] - 0.025)], axis=1)
    return np.concatenate([table, [mouse + [dx, dy, -0.01] for dx in (-0.03, 0, 0.03) for dy in (-0.03, 0, 0.03)]])


def above_mouse_without_lean(b, mouse):
    """approach_object 那一步:不动腰,夹爪尖到鼠标上方 12cm(朝下差十几度)。"""
    up = b.plan_arm_to("right", mouse + [0, 0, 0.12], orientation=("comfort", "keep", "free"),
                       tool_axis=[0, 0, -1.0], allow_body=False)
    assert up["ok"], up.get("error")
    assert b.execute(up)["ok"]


def test_mouse_on_table_never_pushed_horizontally():
    """2026-11-14 真机复现:鼠标上方 12cm,朝下在接触点够不着。以前换成“肩→物体水平”贴着桌面平推、撞桌面;
    现在:物体在台面上不用水平方向,朝下够不着就先动腰再朝下。"""
    mouse = np.array([0.7971, -0.173, 0.7221])
    b, state = make_backend(obstacle_radius_m=0.10, cloud=table_scene(mouse))
    above_mouse_without_lean(b, mouse)
    plan, done = plan_and_run_touch(b, state, mouse)
    assert plan["kind"] == "body_then", plan.get("approach")
    assert np.allclose(plan["after"]["args"]["direction"], [0, 0, -1.0], atol=1e-6)
    assert any("台面" in n for n in plan["direction_notes"])
    assert done["ok"], done["msg"]
    end, R = b.arm.pose("right")
    assert np.linalg.norm(end - mouse) < 0.01
    assert math.degrees(math.acos(-R[2, 2])) < 16              # 从上面下去的,不是侧面平推


def test_mouse_on_table_without_body_fails_instead_of_pushing():
    """不许动腰时,宁可规划失败也不从侧面平推。"""
    mouse = np.array([0.7971, -0.173, 0.7221])
    b, state = make_backend(obstacle_radius_m=0.10, cloud=table_scene(mouse))
    above_mouse_without_lean(b, mouse)
    plan = b.plan_touch("right", mouse, 0.0, [0, 0, -1.0], allow_body=False)
    assert not plan["ok"] or plan["approach"] != "肩→物体水平"


def test_keep_out_blocks_final_segment_through_table():
    """几何检查本身:同样的桌面,水平方向(假装鼠标没放在台面上,强行给水平方向)的候选全部被拒。"""
    mouse = np.array([0.7971, -0.173, 0.7221])
    b, state = make_backend(obstacle_radius_m=0.10, cloud=table_scene(mouse))
    above_mouse_without_lean(b, mouse)
    horizontal = mouse - np.array(b.shoulder_xyz("right"))
    horizontal[2] = 0.0
    horizontal /= np.linalg.norm(horizontal)
    blocked = b.arm.octomap_base()
    from harness.touch_keep_out import outside_object
    blocked = outside_object(blocked, mouse, 0.08)
    found = 0
    for roll in TOUCH["roll_offsets_deg"]:
        pose = b._touch_contact_pose("right", mouse, horizontal, roll)
        if pose is None:
            continue
        for b_cm in TOUCH["backoffs_cm"]:
            cand = b._touch_candidate("right", mouse - b_cm / 100 * horizontal, pose, blocked, mouse, horizontal)
            assert cand in ("ik", "collide", "last", "keep_out"), "水平贴着桌面的最后一段不该通过"
            found += cand == "keep_out"
    assert found > 0


def touch_down_on_mouse(wrench, table_z=None, gap_m=0.0):
    """先动腰到鼠标上方,再 touch 朝下(和第一个测试同一个场景),带六维力替身。"""
    b, state = make_backend(obstacle_radius_m=0.10, wrench=wrench, table_z=table_z)
    lean_above_mouse(b, state)
    plan, done = plan_and_run_touch(b, state, MOUSE, gap_m)
    return b, state, plan, done


def test_force_stops_at_table_then_backs_off_2cm():
    """张开的夹爪罩住鼠标往下:越过鼠标顶面,夹爪尖碰到桌面(力 > 5N)就停,再往回退 2cm。"""
    table_z = MOUSE[2] - 0.03
    b, state, plan, done = touch_down_on_mouse("ok", table_z=table_z)
    assert done["ok"], done["msg"]
    assert done["contact"]["felt"] and done["contact"]["retreat_m"] == 0.02
    end = b.arm.pose("right")[0]
    assert abs(end[2] - (table_z + 0.02)) < 0.008, end[2]      # 停在桌面上方约 2cm(差一个路点以内)
    assert 0.025 < done["contact"]["moved_m"]                    # 确实越过了鼠标顶面(接触点)


def test_force_never_felt_reports_it():
    """多走 5cm 也没感到力:照实说可能没碰到。"""
    b, state, plan, done = touch_down_on_mouse("ok", table_z=None)
    assert done["ok"] and done["contact"]["felt"] is False
    assert "可能没碰到" in done["msg"]
    assert np.linalg.norm(b.arm.pose("right")[0] - (MOUSE + [0, 0, -0.05])) < 0.01


def test_no_force_stream_only_goes_to_contact_point():
    """收不到六维力:退回原来的做法,只走到接触点,不多走。"""
    b, state, plan, done = touch_down_on_mouse("silent", table_z=MOUSE[2] - 0.03)
    assert done["ok"] and done["contact"]["felt"] is None
    assert np.linalg.norm(b.arm.pose("right")[0] - MOUSE) < 0.01


def test_force_stream_lost_midway_stops():
    b, state, plan, done = touch_down_on_mouse("lost")
    assert not done["ok"] and "没更新" in done["msg"]
    assert state["cancel"]


def test_gap_above_zero_does_not_overshoot():
    """gap > 0(停在跟前):不多走,没碰到是正常的。"""
    b, state, plan, done = touch_down_on_mouse("ok", table_z=None, gap_m=0.02)
    assert done["ok"] and done["contact"]["felt"] is False and "可能没碰到" not in done["msg"]
    assert np.linalg.norm(b.arm.pose("right")[0] - (MOUSE + [0, 0, 0.02])) < 0.01


def thick_table_scene(mouse, layers=4):
    """真机的桌面:shadow_cloud 往后补 25cm,桌面在 octomap 里是好几层厚的一块,不是一层。"""
    one = table_scene(mouse)
    table = one[:-9]
    mouse_cells = one[-9:]
    stacked = [table + [0, 0, -0.03 * k] for k in range(layers)]
    return np.concatenate(stacked + [mouse_cells])


def test_force_touch_on_thick_table_goes_straight_down():
    """2026-11-14 真机复现:鼠标上方 12cm、夹爪竖直朝下,touch(gap 0,有六维力)。
    以前要求现在的构型压得到最深点(鼠标顶面下 5cm),差 2cm 就换构型分支,走了 154° 的关节空间 → 没到位。
    现在:不换构型,先多前倾 5°,再竖直往下,碰到桌面停、退 2cm;全程不走关节空间。"""
    mouse = np.array([0.6462, -0.128, 0.7213])
    table_z = mouse[2] - 0.03
    b, state = make_backend(obstacle_radius_m=0.0, cloud=thick_table_scene(mouse), wrench="ok", table_z=table_z)
    above_mouse_without_lean(b, mouse)
    plan, done = plan_and_run_touch(b, state, mouse)
    assert plan["kind"] == "body_then" and "压不到" in plan["summary"]   # 现在姿态压不到桌面:先多前倾
    assert done["ok"], done["msg"]
    assert done["contact"]["felt"]
    assert abs(b.arm.pose("right")[0][2] - (table_z + 0.02)) < 0.008


def test_joint_space_fallback_refuses_big_swing():
    """第一段末端规划不通、关节空间要甩超过 max_joint_fallback_deg:宁可失败。"""
    b, state = make_backend(obstacle_radius_m=0.10, line_works=False)
    b.touch = dict(TOUCH, max_joint_fallback_deg=1)
    plan = b.plan_touch("right", MOUSE, 0.0, [0, 0, -1.0], allow_body=False)
    assert not plan["ok"] and "不走" in plan["error"]
    assert state["joint_plans"] == []
