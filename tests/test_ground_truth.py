"""仿真用的部分(不需要仿真在跑、不需要 ROS 通信):
真值分割 → SAM 3 格式、名字匹配、RosSource 配上分割图、配置叠加、夹爪不连 GDK。

    python3 -m pytest tests/test_ground_truth.py -q
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from harness.backends.gripper_gdk import GdkGripper  # noqa: E402
from harness.perception.ground_truth import label_matches, results_from_seg  # noqa: E402
from harness.perception.rgbd_source import RosSource  # noqa: E402
from run_task import check_sim_domain, merge_config  # noqa: E402

TABLE = [{"index": 1, "name": "mouse_1", "labels": ["mouse", "computer mouse"]},
         {"index": 2, "name": "red_block_1", "labels": ["red block", "block"]},
         {"index": 3, "name": "mouse_2", "labels": ["mouse"]},
         {"index": 4, "name": "cup_1", "labels": ["cup"]}]


def test_label_matching():
    assert label_matches("mouse", ["mouse"])
    assert label_matches("Computer Mouse", ["mouse", "computer mouse"])
    assert label_matches("the mouse", ["mouse"])
    assert label_matches("mice", ["mouse"])
    assert label_matches("red blocks", ["red block"])
    assert label_matches("boxes", ["box"])
    assert not label_matches("keyboard", ["mouse", "computer mouse"])
    assert not label_matches("block", ["red block"])      # 不做部分匹配,和真名一样才算


def test_results_from_seg_format():
    seg = np.zeros((10, 20), np.uint16)
    seg[2:4, 5:9] = 1          # mouse_1:2×4 = 8 像素
    seg[6:9, 1:3] = 3          # mouse_2:3×2 = 6 像素
    seg[0, 0] = 2              # red_block_1
    results = results_from_seg(seg, TABLE, ["mouse", "cup"])
    assert [r["prompt"] for r in results] == ["mouse", "cup"]
    mice = results[0]["instances"]
    assert [i["sim_name"] for i in mice] == ["mouse_1", "mouse_2"]
    assert mice[0]["area_px"] == 8 and mice[0]["box_xyxy"] == [5, 2, 8, 3] and mice[0]["score"] == 1.0
    assert mice[0]["mask"].dtype == bool and mice[0]["mask"].sum() == 8
    assert results[1]["instances"] == []                  # 杯子在表里,但这一帧看不见


class FakeSensor:
    def __init__(self):
        self.node = None
        self.specs = None

    def open(self, specs):
        self.specs = specs

    def close(self):
        pass


def stamped(sec, **fields):
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0)))
    for key, value in fields.items():
        setattr(msg, key, value)
    return msg


def image16(sec, value):
    a = np.full((2, 3), value, np.uint16)
    return stamped(sec, encoding="16UC1", height=2, width=3, step=6, data=a.tobytes())


def tf_msg():
    t = SimpleNamespace(header=SimpleNamespace(frame_id="base_link"), child_frame_id="head_link3",
                        transform=SimpleNamespace(translation=SimpleNamespace(x=0, y=0, z=1),
                                                  rotation=SimpleNamespace(x=0, y=0, z=0, w=1)))
    return SimpleNamespace(transforms=[t])


@pytest.mark.parametrize("order", [("color", "depth", "seg"), ("seg", "color", "depth"), ("color", "seg", "depth")])
def test_ros_source_pairs_seg_in_any_order(order):
    import cv2
    source = RosSource(sensor=FakeSensor(), seg_topic="/sim/camera/head_seg")
    assert any(spec[1] == "/sim/camera/head_seg" for spec in source.sensor.specs)
    ok, jpg = cv2.imencode(".jpg", np.zeros((2, 3, 3), np.uint8))
    source._on_tf(tf_msg())
    messages = {"color": lambda: source._on_color(stamped(5, data=jpg.tobytes())),
                "depth": lambda: source._on_depth(image16(5, 700)),
                "seg": lambda: source._on_seg(image16(5, 2))}
    for name in order[:-1]:
        messages[name]()
        assert source.next(None) is None                  # 没齐不算一帧
    messages[order[-1]]()
    frame = source.next(None)
    assert frame is not None and frame[1][0, 0] == 700
    assert source.last_seg is not None and source.last_seg[0, 0] == 2


def test_ros_source_without_seg_unchanged():
    import cv2
    source = RosSource(sensor=FakeSensor())
    ok, jpg = cv2.imencode(".jpg", np.zeros((2, 3, 3), np.uint8))
    source._on_tf(tf_msg())
    source._on_color(stamped(7, data=jpg.tobytes()))
    source._on_depth(image16(7, 500))
    frame = source.next(None)
    assert frame is not None and source.last_seg is None


def test_merge_config():
    base = {"vision": {"timeout_s": 4.0, "sam3": {"url": "u"}}, "motion": {"gripper": {"interface": "servo"}}}
    over = {"vision": {"segmenter": "ground_truth"}, "motion": {"gripper": {"sim_models": {"right": "omnipicker"}}}}
    merged = merge_config(base, over)
    assert merged["vision"] == {"timeout_s": 4.0, "sam3": {"url": "u"}, "segmenter": "ground_truth"}
    assert merged["motion"]["gripper"] == {"interface": "servo", "sim_models": {"right": "omnipicker"}}
    assert "segmenter" not in base["vision"]              # 原字典不改


def test_sim_refuses_real_robot_domain(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "4")
    with pytest.raises(SystemExit):
        check_sim_domain()
    monkeypatch.delenv("ROS_DOMAIN_ID")
    with pytest.raises(SystemExit):
        check_sim_domain()
    monkeypatch.setenv("ROS_DOMAIN_ID", "77")
    check_sim_domain()


def test_sim_gripper_does_not_touch_gdk():
    gripper = GdkGripper(node=None, cfg={"interface": "servo", "sim_models": {"right": "omnipicker", "left": ""}})
    right = gripper.state("right")
    assert right["ok"] and right["model"] == "omnipicker" and right["error_code"] == 0
    assert gripper._robot is None                         # 没连 GDK
    left = gripper.command("left", "open")
    assert not left["ok"] and "没有识别到夹爪" in left["error"]
