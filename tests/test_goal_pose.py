"""手臂终点姿态:free_goal(只管位置)和 ComfortPose(三角函数舒适姿态)。要 PyKDL + G2 URDF,没有就跳过。

    source /opt/ros/humble/setup.bash
    python3 -m pytest tests/test_goal_pose.py -q
"""
import os

import numpy as np
import pytest

pytest.importorskip("PyKDL")
pytest.importorskip("urdf_parser_py")

from harness.motion.algorithms import ArmSolver  # noqa: E402
from harness.motion.goal_pose import ComfortPose, free_goal, roll_change_deg, roll_locked, tilt_search  # noqa: E402
from harness.motion.kin import arm_kin, body_kin  # noqa: E402

URDF_PATH = os.environ.get("G2_URDF", os.path.expanduser(
    "~/agi@g2_robot/src/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf"))
Q_LEFT = np.array([1.5200, -1.5223, -1.6093, -1.5443, -0.0479, 0.0202, -0.0413])   # 真机实测


@pytest.fixture(scope="module")
def setup():
    if not os.path.exists(URDF_PATH):
        pytest.skip(f"没有 URDF {URDF_PATH}")
    urdf = open(URDF_PATH, encoding="utf-8").read()
    bk = body_kin(urdf)
    pb, Rb = bk.fk(np.zeros(bk.n))
    solver = ArmSolver(arm_kin(urdf, "left"))
    return {"urdf": urdf, "pb": pb, "Rb": Rb, "solver": solver, "comfort": ComfortPose(urdf, "left", solver),
            "up": Rb.T @ np.array([0, 0, 1.0]), "left": Rb.T @ np.array([0, 1.0, 0])}


def to_root(s, xyz_base):
    return s["Rb"].T @ (np.asarray(xyz_base, float) - s["pb"])


def test_geometry_matches_urdf(setup):
    cp = setup["comfort"]
    assert abs(cp.L1 - 0.332) < 0.005 and abs(cp.L2 - 0.278) < 0.005 and abs(cp.c - 0.090) < 0.002


def test_free_goal_reaches_position(setup):
    s = setup
    T = to_root(s, [0.55, 0.15, 0.95])
    q, _ = free_goal(s["solver"], Q_LEFT, T)
    assert q is not None
    assert np.linalg.norm(s["solver"].kin.fk(q)[0] - T) < 0.003


def test_comfort_candidates_are_geometrically_consistent(setup):
    s, cp = setup, setup["comfort"]
    T = to_root(s, [0.55, 0.20, 0.95])
    R_now = s["solver"].kin.fk(Q_LEFT)[1]
    cands = cp.candidates(T, s["up"], s["left"], R_now)
    assert cands
    for c in cands:
        assert abs(np.linalg.norm(c["W"] - T) - cp.c) < 1e-6              # 腕心离末端 = c
        assert abs(np.linalg.norm(c["W"] - cp.S) - c["d"]) < 1e-4        # 腕心离肩 = d(d 存的是 4 位小数)
        assert np.allclose(c["R"].T @ c["R"], np.eye(3), atol=1e-9)      # 正交
        assert np.allclose(c["R"][:, 2], (T - c["W"]) / cp.c, atol=1e-9)  # 工具轴 Z = 腕心→末端
        assert abs(roll_change_deg(R_now, c["R"])) < 1e-6                 # 不绕工具轴自转
        assert abs(np.linalg.norm(c["E"] - cp.S) - cp.L1) < 1e-6         # 肘离肩 = L1


def test_comfort_solution_reaches_and_is_more_comfortable(setup):
    s, cp = setup, setup["comfort"]
    T = to_root(s, [0.55, 0.20, 0.95])
    best, tried = cp.solve(T, Q_LEFT, s["up"], s["left"])
    assert best is not None and tried > 0
    p, R = s["solver"].kin.fk(best["q"])
    assert np.linalg.norm(p - T) < 0.003
    free_q, _ = free_goal(s["solver"], Q_LEFT, T)
    assert best["score"] <= cp.score(free_q)          # 至少不比“只管位置”的解别扭


def test_roll_locked_only_tilts_tool_axis():
    rng = np.random.default_rng(3)
    for _ in range(20):
        R_now = roll_locked(np.eye(3), rng.normal(size=3))               # 随便一个姿态
        z_new = rng.normal(size=3)
        R = roll_locked(R_now, z_new)
        assert np.allclose(R[:, 2], z_new / np.linalg.norm(z_new))
        assert abs(roll_change_deg(R_now, R)) < 1e-6
        assert abs(roll_change_deg(R_now, roll_locked(R_now, z_new, 30)) - 30) < 1e-6


def test_free_and_comfort_keep_roll(setup):
    """放开 pitch / yaw,但手腕不自转。"""
    s, cp = setup, setup["comfort"]
    R_now = s["solver"].kin.fk(Q_LEFT)[1]
    for xyz in ([0.55, 0.20, 0.95], [0.6, 0.0, 1.0], [0.5, 0.35, 0.9]):
        T = to_root(s, xyz)
        q, note = free_goal(s["solver"], Q_LEFT, T)
        if q is not None and not note:
            assert abs(roll_change_deg(R_now, s["solver"].kin.fk(q)[1])) < 1.0
        best, _ = cp.solve(T, Q_LEFT, s["up"], s["left"])
        if best is not None:
            assert abs(roll_change_deg(R_now, s["solver"].kin.fk(best["q"])[1])) < 1.0


def test_tilt_search_retreats_until_gripper_points_down():
    """真机日志 2026-11-12 23:32 的腰 + 右臂关节角:原地夹爪朝下 IK 无解,后撤着转能完全朝下,而且不自转。"""
    import math
    urdf = setup_urdf()
    bk = body_kin(urdf)
    pb, Rb = bk.fk(np.radians([-39.85, 89.84, -49.69, 0, 0]))
    solver = ArmSolver(arm_kin(urdf, "right"))
    q0 = np.radians([-173.49, 91.85, 130.05, -44.01, 152.84, -18.78, -11.43])
    p0, R0 = solver.kin.fk(q0)
    down = Rb.T @ np.array([0, 0, -1.0])
    shoulder = pb + Rb @ solver.kin.segment_positions(np.zeros(7))[1]
    back = shoulder - (pb + Rb @ p0)
    back[2] = 0
    found = tilt_search(solver, q0, p0, R0, down, Rb.T @ (back / np.linalg.norm(back)), 0.30, 0.05)
    assert found is not None and found["fraction"] == 1.0 and 0 < found["retreat_m"] <= 0.30
    R = Rb @ solver.kin.fk(found["q"])[1]
    assert math.degrees(math.acos(-R[2, 2])) < 2.0                       # 离竖直朝下 < 2°
    assert abs(roll_change_deg(R0, solver.kin.fk(found["q"])[1])) < 1.0  # 不自转


def setup_urdf():
    if not os.path.exists(URDF_PATH):
        pytest.skip(f"没有 URDF {URDF_PATH}")
    return open(URDF_PATH, encoding="utf-8").read()
