"""WholeBody 碰撞检查:起点就有、且这次不动的部件之间的接触不算(右手挨着人手时照样能转头);
起点就有、涉及要动的部件的接触(夹着鼠标回初始位姿),只在起点附近 ESCAPE_RAD 以内不算(脱离)。
不连 ROS:用假的 _contacts。

    python3 -m pytest tests/test_whole_body_contacts.py -q
"""
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("moveit_msgs")

from harness.motion.whole_body import ESCAPE_RAD, WholeBody, _moves  # noqa: E402

HAND_CONTACT = ("<octomap>", "arm_r_end_link")


def test_which_links_move():
    assert _moves("head_link3", ["head"]) and not _moves("arm_r_end_link", ["head"])
    assert _moves("arm_r_end_link", ["right_arm"]) and _moves("tcp_r_link", ["right_arm"])
    assert not _moves("<octomap>", ["head", "right_arm"])
    assert _moves("arm_l_link3", ["body"]) and not _moves("base_link", ["body"])


class FakeBody(WholeBody):
    """不建 ROS 节点:碰撞结果按“右手挨着点云”写死,头动到某个角度会撞桌面。"""
    def __init__(self, extra=None):
        self._allowed = set()
        self._escape = set()
        self._start = {}
        self.extra = extra or []

    def _contacts(self, joints):
        pairs = [HAND_CONTACT]
        if joints.get("idx13_head_joint3", 0) > 0.4:
            pairs = pairs + self.extra
        return sorted(pairs), None


def test_head_move_ignores_existing_hand_contact():
    body = FakeBody()
    body._allowed = {HAND_CONTACT}            # plan() 里按“起点已有 + 不动”算出来的
    assert body._valid({"idx13_head_joint3": 0.2})[0]


def test_new_contact_still_blocks():
    body = FakeBody(extra=[("<octomap>", "head_link3")])
    body._allowed = {HAND_CONTACT}
    ok, why = body._valid({"idx13_head_joint3": 0.5})
    assert not ok and "head_link3" in why and "arm_r_end_link" not in why


def test_contact_on_moving_arm_only_ignored_near_start():
    """右臂自己要动时,右手和点云的起点接触只在起点附近放过(先脱离);离起点超过 ESCAPE_RAD 还撞 = 真撞。"""
    assert _moves(HAND_CONTACT[1], ["right_arm"])
    body = FakeBody()
    body._escape = {HAND_CONTACT}
    body._start = {"idx61_arm_r_joint1": 0.0}
    assert body._valid({"idx61_arm_r_joint1": ESCAPE_RAD * 0.5})[0]
    ok, why = body._valid({"idx61_arm_r_joint1": ESCAPE_RAD * 1.5})
    assert not ok and "arm_r_end_link" in why
