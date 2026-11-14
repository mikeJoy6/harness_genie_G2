"""① 有 TCP 的 URDF(end_effector:=true)手臂链末端是 tcp_*_link(法兰沿 z 0.128m);
② ArmMotion.path_clear 起点就接触时允许“脱离”:末端离起点 ESCAPE_M 内的碰撞不算,之后不许再撞。

    source /opt/ros/humble/setup.bash
    python3 -m pytest tests/test_tcp_and_escape.py -q
"""
import os

import numpy as np
import pytest

pytest.importorskip("PyKDL")
pytest.importorskip("rclpy")

from harness.motion.arm_motion import ESCAPE_M, ArmMotion  # noqa: E402
from harness.motion.kin import ChainKin, arm_kin  # noqa: E402

EE_URDF = os.path.expanduser("~/agi@g2_robot/src/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_ee.urdf")
PLAIN_URDF = os.path.expanduser("~/agi@g2_robot/src/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf")


def test_tip_is_tcp_when_urdf_has_it():
    if not (os.path.exists(EE_URDF) and os.path.exists(PLAIN_URDF)):
        pytest.skip("没有 G2 URDF")
    ee, plain = open(EE_URDF).read(), open(PLAIN_URDF).read()
    assert arm_kin(ee, "right").tip == "tcp_r_link"
    assert arm_kin(plain, "right").tip == "arm_r_end_link"
    kin_tcp = arm_kin(ee, "right")
    kin_end = ChainKin(ee, "arm_base_link", "arm_r_end_link")
    q = np.array([0.3, -0.5, 0.2, -1.2, 0.3, 0.4, -0.2])
    p_tcp, R_tcp = kin_tcp.fk(q)
    p_end, R_end = kin_end.fk(q)
    assert np.allclose(R_tcp, R_end)
    assert np.allclose(p_tcp, p_end + 0.128 * R_end[:, 2])          # TCP = 法兰沿自身 z 0.128m


class FakeKin:
    def fk(self, q):
        return np.array([0.0, 0.0, float(q[0])]), np.eye(3)       # 末端高度 = 第 1 个关节值,方便构造路径


class FakeMotion:
    """只给 path_clear 用到的东西:碰撞服务就绪、valid、kins。"""
    def __init__(self, blocked):
        self.blocked = blocked                  # blocked(z) → True 表示这个高度撞
        self.kins = {"right": FakeKin()}

        class Sv:
            def service_is_ready(self):
                return True
        self._sv = Sv()

    def valid(self, arm, q, ignore_cloud=False):
        return not self.blocked(float(q[0]))


def path(z0, z1, n=20):
    return [np.array([z, 0, 0, 0, 0, 0, 0]) for z in np.linspace(z0, z1, n)]


def test_lift_out_of_contact_is_allowed():
    motion = FakeMotion(lambda z: z < 0.03)                         # 起点 0 处夹着东西,3cm 以上就不撞
    assert ArmMotion.path_clear(motion, "right", path(0.0, 0.15))


def test_still_in_contact_after_escape_distance_fails():
    motion = FakeMotion(lambda z: z < ESCAPE_M + 0.03)              # 撞的范围比允许脱离的距离还大
    assert not ArmMotion.path_clear(motion, "right", path(0.0, 0.15))


def test_no_new_collision_after_escaping():
    motion = FakeMotion(lambda z: z < 0.03 or 0.10 < z < 0.12)      # 脱离后又撞上别的
    assert not ArmMotion.path_clear(motion, "right", path(0.0, 0.15))


def test_normal_start_unchanged():
    motion = FakeMotion(lambda z: 0.05 < z < 0.06)                  # 起点不撞、中途撞:照旧拒绝
    assert not ArmMotion.path_clear(motion, "right", path(0.0, 0.15))


# ---------------- ignore_cloud:只有 touch_object 最后一段用,只放过点云接触 ----------------
class Contact:
    def __init__(self, a, b):
        self.contact_body_1, self.contact_body_2 = a, b


class Response:
    def __init__(self, pairs):
        self.valid = not pairs
        self.contacts = [Contact(a, b) for a, b in pairs]


def motion_with_contacts(pairs):
    """ArmMotion.valid 真代码,MoveIt 碰撞服务换成固定回答 pairs。"""
    import threading

    class Sv:
        def service_is_ready(self):
            return True

        def call_async(self, req):
            return Response(pairs)

    class Kin:
        joint_names = ["j1"]

    m = ArmMotion.__new__(ArmMotion)
    m._sv, m.kins, m.cur, m._lock = Sv(), {"right": Kin()}, {"j1": 0.0}, threading.Lock()
    m._wait_future = lambda fut, timeout: fut
    return m


def test_ignore_cloud_only_ignores_octomap():
    cloud = motion_with_contacts([("<octomap>", "tcp_r_link")])
    assert not cloud.valid("right", [0.0])
    assert cloud.valid("right", [0.0], ignore_cloud=True)
    self_hit = motion_with_contacts([("<octomap>", "tcp_r_link"), ("arm_r_link5", "body_link3")])
    assert not self_hit.valid("right", [0.0], ignore_cloud=True)      # 自碰撞照样拦
    table = motion_with_contacts([("table_box", "tcp_r_link")])
    assert not table.valid("right", [0.0], ignore_cloud=True)         # 碰撞体(桌面盒子)照样拦
