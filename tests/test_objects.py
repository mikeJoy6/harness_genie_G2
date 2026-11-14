"""approach_object:手臂末端到 find_objects 找到的物体上方。全部用假机器人,不需要 ROS 和 SAM 3。

    python3 -m pytest tests/test_objects.py -q
"""
import json

import yaml

from harness.backends.fake import FakeBackend
from harness.gateway import Gateway
from harness.loop import run_agent
from harness.tools.objects import make_tools

CFG = yaml.safe_load(open("configs/harness.yaml", encoding="utf-8"))
MOTION = CFG["motion"]
TOOLS = make_tools()
BLOCK = (0.55, -0.25, 0.60)                       # 右手前方桌上的积木
EXPECTED = [[0.55, -0.25, 0.60 + MOTION["standoff_default_m"]]]


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
    return Gateway(backend, MOTION, **kwargs)


def found(block=BLOCK, **kwargs):
    """假机器人 + find_objects(block) 已经找到 block_1。"""
    backend = FakeBackend(sam_objects={"block": [block]}, **kwargs)
    gateway = gateway_for(backend)
    assert gateway.execute("find_objects", {"query": "block"})["ok"]
    return backend, gateway


def test_full_flow_stops_above_object():
    backend = FakeBackend(sam_objects={"block": [BLOCK]})
    llm = ScriptedLLM([call("find_objects", '{"query": "block"}'), call("get_robot_state"),
                       call("approach_object", '{"object_id": "block_1"}'), say("完成")])
    assert run_agent("右手去积木上方", llm, gateway_for(backend), TOOLS) == "完成"
    assert backend.executed == EXPECTED


def test_default_arm_is_right():
    """只有右手装了夹爪:approach_object 不填 arm 就是右臂。"""
    backend, gateway = found()
    assert gateway.execute("approach_object", {"object_id": "block_1"})["arm"] == "right"


def test_cannot_move_without_finding():
    backend = FakeBackend()
    result = gateway_for(backend).execute("approach_object", {"object_id": "block_1"})
    assert not result["ok"] and "find_objects" in result["error"]
    assert backend.executed == []


def test_model_cannot_supply_coordinates():
    backend, gateway = found()
    gateway.execute("approach_object", {"object_id": "block_1", "x": 2.0, "y": 2.0, "z": 0.0, "speed": 9})
    assert backend.executed == EXPECTED


def test_standoff_cannot_touch_object():
    """standoff 按夹爪尖(TCP)算,最小 standoff_min_m;再低就拒绝(碰东西用 touch_object)。"""
    backend, gateway = found()
    too_low = MOTION["standoff_min_m"] * 100 - 1
    assert not gateway.execute("approach_object", {"object_id": "block_1", "standoff_cm": too_low})["ok"]
    assert not gateway.execute("approach_object", {"object_id": "block_1", "standoff_cm": "20"})["ok"]
    assert backend.executed == []


def test_moves_per_command_limited():
    backend, gateway = found()
    limit = MOTION["max_moves_per_command"]
    for _ in range(limit):
        assert gateway.execute("approach_object", {"object_id": "block_1"})["ok"]
    assert "次数已用完" in gateway.execute("approach_object", {"object_id": "block_1"})["error"]
    assert len(backend.executed) == limit


def test_pick_up_sequence_fits_move_limit():
    """“低头找鼠标,拿起来”= 转头 + 上方 + 张开 + 下探 + 闭合 + 上抬 = 6 次运动,不能被次数上限拦住。"""
    assert MOTION["max_moves_per_command"] >= 6


def test_stale_detection_refused():
    backend, gateway = found()
    gateway.objects["block_1"]["time"] -= MOTION["detection_max_age_s"] + 1
    result = gateway.execute("approach_object", {"object_id": "block_1"})
    assert not result["ok"] and "过期" in result["error"]
    assert backend.executed == []


def test_workspace_refused():
    backend, gateway = found(block=(1.9, -0.2, 0.6), end=(1.4, 0.2, 0.9))
    assert "工作范围" in gateway.execute("approach_object", {"object_id": "block_1"})["error"]
    assert backend.executed == []


def test_long_travel_refused():
    backend, gateway = found(block=(1.8, -0.7, 0.4), end=(0.2, 0.75, 2.2))
    assert "移动距离" in gateway.execute("approach_object", {"object_id": "block_1"})["error"]
    assert backend.executed == []


def test_start_below_table_refused():
    backend, gateway = found(end=(0.4, 0.3, 0.60))
    result = gateway.execute("approach_object", {"object_id": "block_1"})
    assert "穿过桌面" in result["error"] and result["executed"] is False
    assert backend.executed == []


def test_user_can_refuse():
    backend = FakeBackend(sam_objects={"block": [BLOCK]})
    gateway = gateway_for(backend, confirm=lambda prompt: False)
    gateway.execute("find_objects", {"query": "block"})
    assert gateway.execute("approach_object", {"object_id": "block_1"})["executed"] is False
    assert backend.executed == []


def test_plan_only_never_moves():
    backend = FakeBackend(sam_objects={"block": [BLOCK]})
    gateway = gateway_for(backend, plan_only=True)
    gateway.execute("find_objects", {"query": "block"})
    assert gateway.execute("approach_object", {"object_id": "block_1"})["executed"] is False
    assert backend.executed == []


def test_planning_failure_reaches_model():
    backend = FakeBackend(sam_objects={"block": [BLOCK]}, plan_error="笛卡尔路径会撞")
    llm = ScriptedLLM([call("find_objects", '{"query": "block"}'),
                       call("approach_object", '{"object_id": "block_1"}'), say("失败了")])
    run_agent("右手去积木上方", llm, gateway_for(backend), TOOLS)
    assert "笛卡尔路径会撞" in json.loads(llm.seen[2][-1]["content"])["error"]
    assert backend.executed == []


def test_no_color_tools_left():
    """2026-11-13:只靠 SAM 3,HSV 颜色定位整套删掉。"""
    names = [t["function"]["name"] for t in TOOLS]
    assert "locate_object" not in names and "get_left_end_pose" not in names
