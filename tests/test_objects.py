import json
import shutil

import numpy as np
import yaml

from harness.backends.color_object import color_mask, find_objects, merge_frames
from harness.backends.fake import FakeBackend
from harness.colors import load_colors, range_from_samples, save_color
from harness.gateway import Gateway
from harness.loop import run_agent
from harness.tools.objects import make_tools

CFG = yaml.safe_load(open("configs/harness.yaml", encoding="utf-8"))
MOTION, VISION = CFG["motion"], CFG["vision"]
COLORS = load_colors("configs/colors.yaml")
TOOLS = make_tools(COLORS)
EXPECTED = [[0.55, 0.25, 0.60 + MOTION["standoff_default_m"]]]


class ScriptedLLM:
    def __init__(self, replies):
        self.replies = list(replies)
        self.seen = []

    def chat(self, messages, tools):
        self.seen.append(list(messages))
        return self.replies.pop(0)


def call(name, arguments="{}"):
    return {"role": "assistant", "content": "", "tool_calls": [
        {"id": "c1", "type": "function", "function": {"name": name, "arguments": arguments}}]}


def say(text):
    return {"role": "assistant", "content": text}


def gateway_for(backend, **kwargs):
    return Gateway(backend, MOTION, COLORS, **kwargs)


def located(**kwargs):
    backend = FakeBackend(**kwargs)
    gateway = gateway_for(backend)
    assert gateway.execute("locate_object", {"color": "red"})["ok"]
    return backend, gateway


def test_full_flow_stops_above_object():
    backend = FakeBackend()
    llm = ScriptedLLM([call("locate_object", '{"color": "green"}'), call("get_left_end_pose"),
                       call("approach_object"), say("完成")])
    assert run_agent("靠近青椒", llm, gateway_for(backend), TOOLS) == "完成"
    assert backend.executed == EXPECTED


def test_unknown_color_refused():
    backend = FakeBackend()
    assert "不认识颜色" in gateway_for(backend).execute("locate_object", {"color": "彩虹"})["error"]


def test_failed_locate_clears_previous_target():
    backend, gateway = located()
    assert not gateway.execute("locate_object", {"color": "blue"})["ok"]
    assert not gateway.execute("approach_object", {})["ok"]
    assert backend.executed == []


def test_cannot_move_without_locating():
    backend = FakeBackend()
    assert not gateway_for(backend).execute("approach_object", {})["ok"]
    assert backend.executed == []


def test_model_cannot_supply_coordinates():
    backend, gateway = located()
    gateway.execute("approach_object", {"x": 2.0, "y": 2.0, "z": 0.0, "speed": 9})
    assert backend.executed == EXPECTED


def test_standoff_cannot_touch_object():
    backend, gateway = located()
    assert not gateway.execute("approach_object", {"standoff_cm": 5})["ok"]
    assert not gateway.execute("approach_object", {"standoff_cm": "20"})["ok"]
    assert backend.executed == []


def test_only_one_move_per_command():
    backend, gateway = located()
    assert gateway.execute("approach_object", {})["ok"]
    assert not gateway.execute("approach_object", {})["ok"]
    assert len(backend.executed) == 1


def test_stale_detection_refused():
    backend, gateway = located()
    gateway.target_time -= MOTION["detection_max_age_s"] + 1
    assert not gateway.execute("approach_object", {})["ok"]
    assert backend.executed == []


def test_workspace_refused():
    backend, gateway = located(block=(1.5, 0.2, 0.6), end=(1.4, 0.2, 0.9))
    assert "工作范围" in gateway.execute("approach_object", {})["error"]
    assert backend.executed == []


def test_long_travel_refused():
    backend, gateway = located(end=(0.2, 0.75, 1.3))
    backend.block = [0.8, -0.2, 0.5]
    gateway.execute("locate_object", {"color": "red"})
    assert "移动距离" in gateway.execute("approach_object", {})["error"]
    assert backend.executed == []


def test_start_below_table_refused():
    backend, gateway = located(end=(0.4, 0.3, 0.60))
    assert "穿过桌面" in gateway.execute("approach_object", {})["error"]
    assert backend.executed == []


def test_user_can_refuse():
    backend = FakeBackend()
    gateway = gateway_for(backend, confirm=lambda prompt: False)
    gateway.execute("locate_object", {"color": "red"})
    assert gateway.execute("approach_object", {})["executed"] is False
    assert backend.executed == []


def test_plan_only_never_moves():
    backend = FakeBackend()
    gateway = gateway_for(backend, plan_only=True)
    gateway.execute("locate_object", {"color": "red"})
    assert gateway.execute("approach_object", {})["executed"] is False
    assert backend.executed == []


def test_planning_failure_reaches_model():
    backend = FakeBackend(plan_error="笛卡尔路径会撞")
    llm = ScriptedLLM([call("locate_object", '{"color": "red"}'), call("approach_object"), say("失败了")])
    run_agent("靠近积木", llm, gateway_for(backend), TOOLS)
    assert "笛卡尔路径会撞" in json.loads(llm.seen[2][-1]["content"])["error"]
    assert backend.executed == []


def scene(objects):
    """objects:[(x, y, 边长像素, BGR), ...]"""
    bgr = np.full((400, 640, 3), 200, np.uint8)
    depth = np.full((400, 640), 1.0, np.float32)
    points = np.zeros((400, 640, 3), np.float32)
    points[..., 0], points[..., 1], points[..., 2] = 0.5, 0.2, 0.6
    for x, y, size, color in objects:
        bgr[y:y + size, x:x + size] = color
    return bgr, depth, points


RED, GREEN, BLUE = (0, 0, 220), (40, 180, 40), (220, 40, 40)


def test_each_preset_finds_only_its_color():
    args = scene([(100, 100, 16, RED), (300, 100, 16, GREEN), (500, 100, 16, BLUE)])
    for name, x in (("red", 100), ("green", 300), ("blue", 500)):
        found, _ = find_objects(*args, 309.0, COLORS[name], VISION)
        assert len(found) == 1 and found[0]["x"] == x
        assert np.allclose(found[0]["xyz_m"], [0.5, 0.2, 0.6])


def test_large_object_rejected():
    found, rejected = find_objects(*scene([(200, 100, 150, RED)]), 309.0, COLORS["red"], VISION)
    assert found == [] and "尺寸" in rejected[0]


def test_color_without_depth_rejected():
    bgr, depth, points = scene([(300, 200, 16, RED)])
    depth[:] = 0
    found, rejected = find_objects(bgr, depth, points, 309.0, COLORS["red"], VISION)
    assert found == [] and "有效深度" in rejected[0]


def test_highlight_does_not_split_object():
    bgr, depth, points = scene([(300, 200, 30, GREEN)])
    bgr[212:215, 300:330] = (255, 255, 255)
    found, _ = find_objects(bgr, depth, points, 309.0, COLORS["green"], VISION)
    assert len(found) == 1


def test_two_objects_picks_closest_to_arm():
    frames = [[{"xyz_m": [1.0, 0, 0]}, {"xyz_m": [0.5, 0, 0]}]] * 5
    block, error = merge_frames(frames, 3, 0.02, [0.4, 0, 0])
    assert error is None and block["xyz_m"] == [0.5, 0, 0] and block["candidates"] == 2
    assert np.isclose(block["distance_to_left_end_m"], 0.1)


def test_closest_changes_with_arm_position():
    frames = [[{"xyz_m": [1.0, 0, 0]}, {"xyz_m": [0.5, 0, 0]}]] * 5
    block, _ = merge_frames(frames, 3, 0.02, [1.1, 0, 0])
    assert block["xyz_m"] == [1.0, 0, 0]


def test_equally_close_objects_flipping_rejected():
    frames = [[{"xyz_m": [0.0, 0, 0]}], [{"xyz_m": [0.1, 0, 0]}], [{"xyz_m": [0.2, 0, 0]}]]
    block, error = merge_frames(frames, 3, 0.02, [0, 0, 0])
    assert block is None and "不稳定" in error


def test_too_few_frames_rejected():
    block, error = merge_frames([[], [], [{"xyz_m": [0, 0, 0]}]], 3, 0.02, [0, 0, 0])
    assert block is None and "1 帧" in error


def test_samples_give_single_range():
    ranges = range_from_samples([[60, 150, 120], [64, 170, 140]])
    assert ranges == [[[52, 110, 70], [72, 210, 190]]]


def test_hue_wrapping_around_zero_splits():
    ranges = range_from_samples([[2, 200, 150], [176, 200, 150]])
    assert ranges == [[[168, 160, 100], [179, 240, 200]], [[0, 160, 100], [10, 240, 200]]]


def test_gray_ignores_hue():
    ranges = range_from_samples([[10, 5, 90], [150, 10, 100]])
    assert ranges[0][0][0] == 0 and ranges[0][1][0] == 179 and len(ranges) == 1


def test_black_is_separable_from_bright_colors():
    ranges = range_from_samples([[0, 0, 20], [0, 0, 25]])
    bgr = np.full((40, 40, 3), 220, np.uint8)
    bgr[10:30, 10:30] = (22, 22, 22)
    assert color_mask(bgr, ranges)[20, 20] > 0 and color_mask(bgr, ranges)[2, 2] == 0


def test_save_color_keeps_presets(tmp_path):
    path = tmp_path / "colors.yaml"
    shutil.copy("configs/colors.yaml", path)
    save_color(path, "青椒", [[[40, 60, 40], [80, 255, 255]]])
    colors = load_colors(path)
    assert colors["青椒"] == [[[40, 60, 40], [80, 255, 255]]] and "red" in colors
    assert path.read_text(encoding="utf-8").startswith("#")
