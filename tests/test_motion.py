"""运控代码(harness/motion/)的离线测试,复制自 agi@g2_robot 的 test_algorithms.py。

需要 ROS 环境(PyKDL、urdf_parser_py)和 G2 的 URDF;找不到就跳过:
    source /opt/ros/humble/setup.bash
    G2_URDF=/路径/G2_t2_crsB.urdf python3 -m pytest tests/test_motion.py -q
默认读 ~/agi@g2_robot/src/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf。
"""
import os

import numpy as np
import pytest

pytest.importorskip("PyKDL")
pytest.importorskip("urdf_parser_py")

from harness.motion.algorithms import (  # noqa: E402
    ArmSolver, path_sane, rotvec, rpy_to_mat, mat_to_rpy, POS_TOL, ROT_TOL)
from harness.motion.kin import arm_kin, body_kin  # noqa: E402

URDF_PATH = os.environ.get("G2_URDF", os.path.expanduser(
    "~/agi@g2_robot/src/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf"))

# 真机实测构型（2026-11-01 /gdk/joint_state）
Q_REAL = {
    "left": np.array([1.5200, -1.5223, -1.6093, -1.5443, -0.0479, 0.0202, -0.0413]),
    "right": np.array([-1.5676, -1.5662, 1.5707, -1.5708, 0.0001, -0.0025, -0.0006]),
}


@pytest.fixture(scope="module")
def urdf():
    if not os.path.exists(URDF_PATH):
        pytest.skip(f"没有 URDF:{URDF_PATH}")
    with open(URDF_PATH) as f:
        return f.read()


def test_chains(urdf):
    for arm in ("left", "right"):
        kin = arm_kin(urdf, arm)
        assert kin.n == 7
        assert all(("_arm_l_" if arm == "left" else "_arm_r_") in j for j in kin.joint_names)
    assert body_kin(urdf).joint_names == [f"idx0{i}_body_joint{i}" for i in range(1, 6)]


def test_rpy_roundtrip():
    for rpy in ((0.1, -0.4, 2.0), (-1.2, 0.3, -0.7)):
        assert np.allclose(mat_to_rpy(rpy_to_mat(*rpy)), rpy, atol=1e-9)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_ik_random_reachable(urdf, arm):
    """随机可达位姿（由随机构型 FK 得到）都要解出来，且 FK 复核在容差内、不超限位。"""
    kin = arm_kin(urdf, arm)
    S = ArmSolver(kin, seed=1)
    rng = np.random.default_rng(42)
    for _ in range(10):
        p, R = kin.fk(rng.uniform(kin.lower * 0.7, kin.upper * 0.7))
        q = S.ik(p, "full", R, None, Q_REAL[arm])
        assert q is not None
        pq, Rq = kin.fk(q)
        assert np.linalg.norm(pq - p) <= POS_TOL
        assert np.linalg.norm(rotvec(R @ Rq.T)) <= ROT_TOL
        assert np.all(q >= kin.lower - 1e-6) and np.all(q <= kin.upper + 1e-6)


@pytest.mark.parametrize("arm", ["right"])
@pytest.mark.parametrize("d", [(0.1, 0, 0), (-0.1, 0, 0), (0, 0.1, 0), (0, -0.1, 0), (0, 0, 0.1), (0, 0, -0.1)])
def test_cartesian_10cm_no_big_loop(urdf, arm, d):
    """从真机构型沿 6 个方向直线走 10cm（保持姿态）：要到位，且路径过防大回环检查。"""
    kin = arm_kin(urdf, arm)
    S = ArmSolver(kin)
    q0 = Q_REAL[arm]
    p0, R0 = kin.fk(q0)
    path, ok, ep, er, _ = S.solve(q0, p0 + np.array(d), "full", R0)
    assert ok, f"ep={ep}, er={er}"
    assert path_sane(path, 0.1, "t") == ""


def test_axis_mode(urdf):
    """axis 模式：只约束末端 X 轴方向，滚转自由。"""
    kin = arm_kin(urdf, "left")
    S = ArmSolver(kin, seed=3)
    p, R = kin.fk(Q_REAL["left"] + 0.2)
    q = S.ik(p, "axis", None, R[:, 0], Q_REAL["left"])
    assert q is not None
    pq, Rq = kin.fk(q)
    assert np.linalg.norm(pq - p) <= POS_TOL
    assert float(Rq[:, 0] @ R[:, 0]) >= np.cos(ROT_TOL)


def _planner_names():
    from harness.motion.planners import discover
    algos, errors = discover()
    assert not errors, errors
    return list(algos)


def test_planner_registry():
    from harness.motion.planners import DEFAULT
    assert DEFAULT in _planner_names()


@pytest.mark.parametrize("name", _planner_names())
def test_planner_detour_around_post(urdf, name):
    """每个绕障规划器：右臂右移 20cm，中间立一根细柱(顶比手高 5cm)——直线撞，要绕行；
    结果要到位、全程不撞（起点除外）；末端空间的规划器还要过 path_sane（不抡大圈）。"""
    from harness.motion.planners import PlanRequest, discover
    kin = arm_kin(urdf, "right")
    bpb, bRb = body_kin(urdf).fk(np.array([-0.3766, 0.7690, -0.2006, 0, 0]))
    to_root = lambda p: bRb.T @ (np.asarray(p) - bpb)
    to_base = lambda p: bRb @ p + bpb
    q0 = Q_REAL["right"]
    p0, R0 = kin.fk(q0)
    p0b = to_base(p0)
    c, h = p0b + np.array([0.0, -0.10, -0.035]), np.array([0.02, 0.02, 0.085])
    grid = np.stack(np.meshgrid(*[np.arange(-x, x + 1e-9, 0.03) for x in h], indexing="ij"), -1).reshape(-1, 3) + c

    def valid(q):
        P = kin.segment_positions(q)
        pts = [to_base(P[0])] + [to_base(a + s * (b - a)) for a, b in zip(P[:-1], P[1:]) for s in np.linspace(0, 1, 6)[1:]]
        return not np.all(np.abs(np.array(pts) - c) <= h + 0.06, axis=-1).any()

    pg = to_root(p0b + np.array([0.0, -0.20, 0.0]))
    req = PlanRequest(solver=ArmSolver(kin), valid=valid, q0=q0, p_goal=pg, R_goal=R0,
                      obstacles=np.array([to_root(p) for p in grid]), up=bRb.T @ np.array([0.0, 0.0, 1.0]))
    res = discover()[0][name].factory().plan(req)
    assert res.ok, res.note
    jp, wps = res.joint_path, res.waypoints
    assert len(wps) > 2, "应该绕行，不是直线"
    assert np.linalg.norm(kin.fk(jp[-1])[0] - pg) <= POS_TOL
    assert all(valid(q) for q in jp[1:])
    if name != "joint_rrt":                     # 关节空间对照组不保证末端路线
        length = sum(np.linalg.norm(b - a) for a, b in zip(wps[:-1], wps[1:]))
        assert path_sane(jp, length, "") == ""


def test_cart_profile_limits(urdf):
    """V 形笛卡尔路径（两段 15cm 直线带拐角）的变速：首尾不动，按 JTC 三次样条算的关节速度/加速度、末端速度都不超限。"""
    from urdf_parser_py.urdf import URDF
    from harness.motion import arm_motion as A
    kin = arm_kin(urdf, "right")
    S = ArmSolver(kin)
    robot = URDF.from_xml_string(urdf)
    vel = np.array([robot.joint_map[j].limit.velocity for j in kin.joint_names])
    q0 = Q_REAL["right"]
    p0, R0 = kin.fk(q0)
    path1, ok1, *_ = S.solve(q0, p0 + [0.15, 0, 0], "full", R0)
    path2, ok2, *_ = S.solve(path1[-1], p0 + [0.15, 0.0, 0.15], "full", R0)
    assert ok1 and ok2
    path = list(path1) + list(path2[1:])
    tt, P, V = A.cart_samples(*A.cart_profile(kin, path, vel))
    assert np.allclose(P[0], path[0]) and np.allclose(P[-1], path[-1])
    vel_pk, acc_pk = A.spline_peaks(tt, P, V)
    assert np.all(vel_pk <= vel * 1.02)
    assert acc_pk.max() <= A.CART_JOINT_ACCEL * 1.001, acc_pk.max()
    ee = np.array([kin.fk(q)[0] for q in P])
    assert np.max(np.linalg.norm(np.diff(ee, axis=0), axis=1) / np.diff(tt)) <= A.CART_SPEED * 1.05
