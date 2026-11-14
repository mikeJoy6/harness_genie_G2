"""SAM 3 找物体 + 转头看物体:find_objects / look_at_object。

前半部分用假机器人,不需要 ROS 和 SAM 3 服务;最后的 HeadAim 测试要 PyKDL 和 G2 URDF,没有就跳过。

    python3 -m pytest tests/test_sam3_look.py -q
"""
import math
import os
import time

import numpy as np
import pytest
import yaml

from harness.backends.fake import FakeBackend
from harness.gateway import Gateway
from harness.loop import run_agent
from harness.perception.sam3_objects import objects_from_masks, save_segmentation, slug
from harness.tools.objects import make_tools

CFG = yaml.safe_load(open("configs/harness.yaml", encoding="utf-8"))
MOTION = CFG["motion"]
HEAD = ("idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3")


def gateway_for(backend=None, **kwargs):
    backend = backend or FakeBackend()
    return backend, Gateway(backend, MOTION, **kwargs)


class ScriptedLLM:
    def __init__(self, replies):
        self.replies = list(replies)

    def chat(self, messages, tools):
        return self.replies.pop(0)


def call(name, arguments):
    return {"role": "assistant", "content": "", "tool_calls": [
        {"id": "c1", "type": "function", "function": {"name": name, "arguments": arguments}}]}


# ---------------- find_objects ----------------
def test_find_objects_sorted_nearest_first_and_stored():
    _, gateway = gateway_for()
    result = gateway.execute("find_objects", {"query": "phone"})
    assert result["ok"] and result["count"] == 2
    ids = [o["id"] for o in result["objects"]]
    assert ids == ["phone_1", "phone_2"]
    assert result["objects"][0]["distance_m"] <= result["objects"][1]["distance_m"]
    assert set(gateway.objects) == {"phone_1", "phone_2"}


def test_find_objects_rejects_chinese_query():
    backend, gateway = gateway_for()
    result = gateway.execute("find_objects", {"query": "手机"})
    assert not result["ok"] and "英文" in result["error"]
    assert backend.segment_calls == []


def test_find_objects_nothing_found():
    _, gateway = gateway_for()
    result = gateway.execute("find_objects", {"query": "banana"})
    assert not result["ok"] and "没有找到" in result["error"]


def test_find_objects_service_down():
    _, gateway = gateway_for(FakeBackend(sam_error="连不上 SAM 3 服务"))
    result = gateway.execute("find_objects", {"query": "phone"})
    assert not result["ok"] and "SAM 3" in result["error"]


def test_refind_drops_old_ids():
    backend, gateway = gateway_for()
    gateway.execute("find_objects", {"query": "phone"})
    backend.sam_objects["phone"] = [(0.7, 0.0, 0.8)]
    gateway.execute("find_objects", {"query": "phone"})
    assert set(gateway.objects) == {"phone_1"}


# ---------------- look_at_object ----------------
def test_look_at_moves_only_head_and_checks_after():
    backend, gateway = gateway_for()
    gateway.execute("find_objects", {"query": "phone"})
    result = gateway.execute("look_at_object", {"object_id": "phone_1"})
    assert result["ok"] and result["executed"], result
    assert set(backend.executed[-1]) == set(HEAD)                 # 只动头
    assert backend.segment_calls[-1] == ("phone", "_after")       # 转完又分割一次,存成 _after
    offset = result["check_after"]["offset_from_center_px"]
    assert abs(offset[0]) < 1 and abs(offset[1]) < 1


def test_look_at_plan_only_does_not_move():
    backend, gateway = gateway_for(plan_only=True)
    gateway.execute("find_objects", {"query": "phone"})
    result = gateway.execute("look_at_object", {"object_id": "phone_1"})
    assert result["ok"] and not result["executed"]
    assert backend.executed == [] and "check_after" not in result


def always_no(prompt):
    return False


def test_look_at_user_refuses():
    backend, gateway = gateway_for(confirm=always_no)
    gateway.execute("find_objects", {"query": "phone"})
    result = gateway.execute("look_at_object", {"object_id": "phone_1"})
    assert not result["ok"] and backend.executed == []


def test_look_at_unknown_id():
    _, gateway = gateway_for()
    result = gateway.execute("look_at_object", {"object_id": "phone_1"})
    assert not result["ok"] and "find_objects" in result["error"]


def test_look_at_expired():
    _, gateway = gateway_for()
    gateway.execute("find_objects", {"query": "phone"})
    gateway.objects["phone_1"]["time"] = time.monotonic() - MOTION["detection_max_age_s"] - 1
    result = gateway.execute("look_at_object", {"object_id": "phone_1"})
    assert not result["ok"] and "过期" in result["error"]


def test_look_at_already_centered():
    backend, gateway = gateway_for()
    gateway.execute("find_objects", {"query": "phone"})
    gateway.execute("look_at_object", {"object_id": "phone_1"})
    gateway.execute("find_objects", {"query": "phone"})
    result = gateway.execute("look_at_object", {"object_id": "phone_1"})
    assert result["ok"] and not result["executed"] and "中央" in result["note"]


def test_agent_look_at_nearest_phone():
    """“看向最近的手机” → find_objects(phone) → look_at_object(phone_1)。"""
    backend, gateway = gateway_for()
    llm = ScriptedLLM([call("find_objects", '{"query": "phone"}'),
                       call("look_at_object", '{"object_id": "phone_1"}'),
                       {"role": "assistant", "content": "完成"}])
    assert run_agent("看向最近的手机", llm, gateway, make_tools()) == "完成"
    assert backend.executed and set(backend.executed[-1]) == set(HEAD)


def test_tools_listed():
    names = [t["function"]["name"] for t in make_tools()]
    assert "find_objects" in names and "look_at_object" in names


# ---------------- 掩码 + 深度 → 物体(纯计算) ----------------
def _fake_scene():
    h, w = 40, 60
    depth = np.zeros((h, w), np.float32)
    points = np.zeros((h, w, 3), np.float32)
    near = np.zeros((h, w), bool)
    far = np.zeros((h, w), bool)
    near[5:15, 5:15] = True
    far[20:30, 30:40] = True
    depth[near], depth[far] = 0.6, 1.2
    points[near] = (0.6, 0.1, 0.8)
    points[far] = (1.2, -0.2, 0.8)
    results = [{"prompt": "cell phone", "instances": [
        {"score": 0.9, "box_xyxy": [30, 20, 40, 30], "area_px": 100, "mask": far},
        {"score": 0.8, "box_xyxy": [5, 5, 15, 15], "area_px": 100, "mask": near}]}]
    return results, depth, points


def test_objects_from_masks_nearest_first():
    results, depth, points = _fake_scene()
    objects, rejected = objects_from_masks(results, depth, points, (0, 0, 0), (30, 20))
    assert [o["id"] for o in objects] == ["cell_phone_1", "cell_phone_2"]
    assert objects[0]["xyz_m"] == [0.6, 0.1, 0.8]
    assert objects[0]["offset_from_center_px"] == [-20.5, -10.5]     # 像素中心 (9.5, 9.5) - 主点 (30, 20)
    assert rejected == []


def test_objects_without_depth_rejected():
    results, depth, points = _fake_scene()
    depth[:] = 0
    objects, rejected = objects_from_masks(results, depth, points, (0, 0, 0), (30, 20))
    assert objects == [] and len(rejected) == 2


def test_save_segmentation_writes_log(tmp_path):
    results, depth, points = _fake_scene()
    objects, _ = objects_from_masks(results, depth, points, (0, 0, 0), (30, 20))
    bgr = np.zeros((40, 60, 3), np.uint8)
    folder = save_segmentation(tmp_path, "cell phone", bgr, depth * 1000, results, objects, {"segment_ms": 1})
    names = sorted(p.name for p in folder.iterdir())
    assert names == ["color.jpg", "depth.png", "mask_cell_phone_1.png", "mask_cell_phone_2.png",
                     "overlay.jpg", "result.json"]
    assert folder.name.endswith("_cell_phone") and slug("Cell Phone!") == "cell_phone"


# ---------------- HeadAim(要 PyKDL + URDF) ----------------
URDF_PATH = os.environ.get("G2_URDF", os.path.expanduser(
    "~/agi@g2_robot/src/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf"))


@pytest.fixture(scope="module")
def head_aim():
    pytest.importorskip("PyKDL")
    pytest.importorskip("urdf_parser_py")
    if not os.path.exists(URDF_PATH):
        pytest.skip(f"没有 URDF {URDF_PATH}")
    from urdf_parser_py.urdf import URDF
    from harness.perception.head_aim import HeadAim
    text = open(URDF_PATH, encoding="utf-8").read()
    robot = URDF.from_xml_string(text)
    limits = {j.name: (j.limit.lower, j.limit.upper) for j in robot.joints if j.limit is not None}
    aim = HeadAim(text)
    zero = {name: 0.0 for name in aim.kin.joint_names}
    return aim, limits, zero


def test_head_aim_centers_reachable_target(head_aim):
    aim, limits, zero = head_aim
    target = [0.9, 0.2, 1.0]                     # 前方偏左、比桌面高:不碰限位
    angles, residual, at_limit = aim.solve(zero, target, limits)
    assert not at_limit
    assert max(abs(v) for v in residual) < 0.1
    assert angles["idx11_head_joint1"] > 0       # 目标在左边 → 向左转(正)
    assert angles["idx12_head_joint2"] == 0.0    # 不歪头
    px = aim.pixels_off_center({**zero, **angles}, target)
    assert abs(px[0]) < 1 and abs(px[1]) < 1


def test_head_aim_reports_limit(head_aim):
    aim, limits, zero = head_aim
    angles, residual, at_limit = aim.solve(zero, [0.3, 0.9, 0.7], limits)   # 左后方,头转不到
    assert at_limit and abs(residual[0]) > 5
    low, high = limits["idx11_head_joint1"]
    assert low < angles["idx11_head_joint1"] < high


# ---------------- approach_object(object_id):手臂去 SAM 3 找到的物体上方 ----------------
RIGHT_PHONE = {"phone": [(0.75, -0.15, 0.71)]}       # 机器人右前方,和真机那次差不多


def test_approach_found_object_with_right_arm():
    backend, gateway = gateway_for(FakeBackend(sam_objects=RIGHT_PHONE, end=(0.59, 0.42, 0.95)))
    gateway.execute("find_objects", {"query": "phone"})
    phone = gateway.objects["phone_1"]["xyz_m"]
    result = gateway.execute("approach_object", {"object_id": "phone_1", "arm": "right", "standoff_cm": 30})
    assert result["ok"] and result["executed"], result
    assert backend.executed[-1] == [phone[0], phone[1], phone[2] + 0.30]


def test_approach_unknown_object_id():
    _, gateway = gateway_for()
    result = gateway.execute("approach_object", {"object_id": "phone_1"})
    assert not result["ok"] and "find_objects" in result["error"]


def test_agent_right_arm_over_nearest_phone():
    backend, gateway = gateway_for(FakeBackend(sam_objects=RIGHT_PHONE, end=(0.59, 0.42, 0.95)))
    llm = ScriptedLLM([call("find_objects", '{"query": "phone"}'),
                       call("approach_object", '{"object_id": "phone_1", "arm": "right", "standoff_cm": 30}'),
                       {"role": "assistant", "content": "完成"}])
    assert run_agent("右臂末端抵达最近的手机上空30厘米处", llm, gateway, make_tools()) == "完成"
    assert backend.ends["right"]["xyz_m"] == [0.75, -0.15, 0.71 + 0.30]


# ---------------- point_at_object:夹爪指向物体 ----------------
HAND = {"hand": [(0.80, -0.30, 1.00)]}


def test_point_at_hand_rotates_in_place_toward_it():
    backend, gateway = gateway_for(FakeBackend(sam_objects=HAND, end=(0.59, 0.42, 0.95)))
    gateway.execute("find_objects", {"query": "hand"})
    start = backend.end_pose("right")["xyz_m"]
    result = gateway.execute("point_at_object", {"object_id": "hand_1", "arm": "right"})
    assert result["ok"] and result["executed"], result
    gap = [h - s for h, s in zip(HAND["hand"][0], start)]
    n = sum(v * v for v in gap) ** 0.5
    assert np.allclose(backend.last_tool_axis, [v / n for v in gap])   # 工具轴 = 末端 → 手
    assert backend.last_orientation == ("axis",)
    assert backend.executed[-1] == start                           # 末端位置不动,只转方向


def test_point_refuses_when_too_close():
    backend, gateway = gateway_for(FakeBackend(sam_objects={"hand": [(0.62, -0.42, 0.95)]}, end=(0.59, 0.42, 0.95)))
    gateway.execute("find_objects", {"query": "hand"})
    result = gateway.execute("point_at_object", {"object_id": "hand_1", "arm": "right"})
    assert not result["ok"] and "太近" in result["error"] and backend.executed == []


def test_agent_point_right_gripper_at_my_hand():
    """“让右夹爪指向我的手” → find_objects(hand) → point_at_object(hand_1, right),不问颜色。"""
    backend, gateway = gateway_for(FakeBackend(sam_objects=HAND, end=(0.59, 0.42, 0.95)))
    llm = ScriptedLLM([call("find_objects", '{"query": "hand"}'),
                       call("point_at_object", '{"object_id": "hand_1", "arm": "right"}'),
                       {"role": "assistant", "content": "完成"}])
    assert run_agent("让右夹爪指向我的手", llm, gateway, make_tools()) == "完成"
    assert backend.executed


def test_find_objects_reports_reach():
    """真机那次:两个拳头,远的那个 x=1.21 两只手都够不着,要让模型看得出来。"""
    backend, gateway = gateway_for(FakeBackend(sam_objects={"fist": [(0.85, -0.12, 0.99), (1.21, -0.37, 0.97)]}))
    result = gateway.execute("find_objects", {"query": "fist"})
    near, far = result["objects"]
    assert "right" in near["reachable_by"]
    assert far["reachable_by"] == []


def test_approach_reports_tool_axis_miss():
    """严格朝下够不着、差几度以内接受时,结果里要写实际差几度,模型如实告诉用户。"""
    class AlmostDown(FakeBackend):
        def plan_arm_to(self, *a, **k):
            plan = super().plan_arm_to(*a, **k)
            plan["tool_axis_miss_deg"] = 6.0
            return plan
    backend, gateway = gateway_for(AlmostDown(sam_objects=RIGHT_PHONE, end=(0.59, 0.42, 0.95)))
    gateway.execute("find_objects", {"query": "phone"})
    result = gateway.execute("approach_object", {"object_id": "phone_1", "arm": "right", "tool_axis": "down"})
    assert result["ok"] and result["tool_axis_miss_deg"] == 6.0


# ---------------- touch_object:击掌、碰拳 ----------------
def test_touch_hand_uses_touch_chain():
    """touch_object 走 backend.plan_touch(唯一不查点云的链路):接近方向 = 末端 → 手,夹爪尖停在手前 2cm。"""
    backend, gateway = gateway_for(FakeBackend(sam_objects=HAND, end=(0.59, 0.42, 0.95)))
    gateway.execute("find_objects", {"query": "hand"})
    start = backend.end_pose("right")["xyz_m"]
    result = gateway.execute("touch_object", {"object_id": "hand_1", "arm": "right"})
    assert result["ok"] and result["executed"], result
    hand = np.array(HAND["hand"][0])
    d = (hand - start) / np.linalg.norm(hand - start)
    gap = MOTION["touch"]["default_gap_cm"] / 100
    assert np.allclose(backend.last_touch["direction"], d)
    assert np.allclose(backend.executed[-1], hand - gap * d)               # 夹爪尖在手前 2cm
    assert abs(result["tip_to_object_m"] - gap) < 1e-6
    assert result["unchecked_m"] == 0.05 and "只放过物体附近的点云" in result["note"]


def test_only_touch_skips_cloud_check():
    """approach_object / move_arm_end 失败就是失败,不会偷偷不查点云贴过去。"""
    backend, gateway = gateway_for(FakeBackend(sam_objects=HAND, end=(0.59, 0.42, 0.95)))
    gateway.execute("find_objects", {"query": "hand"})
    gateway.execute("approach_object", {"object_id": "hand_1"})
    gateway.execute("move_arm_end", {"arm": "right", "offset_cm": [0, 0, -5]})
    assert not hasattr(backend, "last_touch")


def test_agent_high_five_without_asking():
    """“和我击掌” → find_objects(hand) → touch_object(hand_1, right)。"""
    backend, gateway = gateway_for(FakeBackend(sam_objects=HAND, end=(0.59, 0.42, 0.95)))
    llm = ScriptedLLM([call("find_objects", '{"query": "hand"}'),
                       call("touch_object", '{"object_id": "hand_1", "arm": "right"}'),
                       {"role": "assistant", "content": "击掌完成"}])
    assert run_agent("和我击掌", llm, gateway, make_tools()) == "击掌完成"
    assert backend.executed


def test_touch_gap_one_cm():
    """“指着粉色物体,离它 1 厘米” → touch_object(gap_cm=1):夹爪尖停在 1cm 外。"""
    backend, gateway = gateway_for(FakeBackend(sam_objects=HAND, end=(0.59, 0.42, 0.95)))
    gateway.execute("find_objects", {"query": "hand"})
    result = gateway.execute("touch_object", {"object_id": "hand_1", "arm": "right", "gap_cm": 1})
    assert result["ok"] and abs(result["tip_to_object_m"] - 0.01) < 1e-6
    assert not gateway.execute("touch_object", {"object_id": "hand_1", "arm": "right", "gap_cm": 11})["ok"]


def test_tool_texts_steer_the_model():
    """工具说明里要让模型知道:gap_cm 可以小于 2、找物体先用 find_objects。"""
    tools = {t["function"]["name"]: t["function"] for t in make_tools()}
    gap = tools["touch_object"]["parameters"]["properties"]["gap_cm"]["description"]
    assert "0~10" in gap and "1 厘米" in gap
    assert "locate_object" not in tools


# ---------------- 拍照 ----------------
def test_take_photo_default_wide():
    backend, gateway = gateway_for()
    gateway.photo_cfg = {"dir": "photos", "default_camera": "head_wide", "cameras": {"head_wide": "/x", "head": "/y"}}
    result = gateway.execute("take_photo", {})
    assert result["ok"] and result["camera"] == "head_wide" and result["path"].endswith(".jpg")
    assert not gateway.execute("take_photo", {"camera": "tail"})["ok"]


def test_look_at_group_center_then_photo():
    """“给我们这群人拍照” → find_objects(person) → look_at_object(object_ids=[...]) → take_photo。"""
    people = {"person": [(1.0, 0.3, 1.0), (1.2, -0.3, 1.0)]}
    backend, gateway = gateway_for(FakeBackend(sam_objects=people))
    llm = ScriptedLLM([call("find_objects", '{"query": "person"}'),
                       call("look_at_object", '{"object_ids": ["person_1", "person_2"]}'),
                       call("take_photo", '{}'),
                       {"role": "assistant", "content": "拍好了"}])
    assert run_agent("给我们这群人拍照", llm, gateway, make_tools()) == "拍好了"
    yaw = backend.executed[-1]["idx11_head_joint1"]
    assert abs(yaw - math.atan2(0.0 - 0.0, 1.1 - 0.2)) < 1e-6        # 对准两人中间 (1.1, 0, 1.0)
    assert backend.photos == ["head_wide"]


def test_body_assist_is_reported():
    """手臂够不着、程序先动腰:结果里写明前倾多少度,模型如实告诉用户。"""
    class Leaning(FakeBackend):
        def plan_arm_to(self, *a, **k):
            plan = super().plan_arm_to(*a, **k)
            plan.update(kind="body_then", lean_deg=10, turn_deg=0)
            return plan
    backend, gateway = gateway_for(Leaning(sam_objects=RIGHT_PHONE, end=(0.59, 0.42, 0.95)))
    gateway.execute("find_objects", {"query": "phone"})
    result = gateway.execute("approach_object", {"object_id": "phone_1", "arm": "right", "tool_axis": "down"})
    assert result["ok"] and result["body_assist"] == {"lean_deg": 10, "turn_deg": 0} and "前倾 10°" in result["note"]
