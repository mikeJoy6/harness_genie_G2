"""scripts/run_task.py 的 Session:交互模式下多条指令共用一个 Gateway,并把前几条的问答带给模型。不需要 ROS。

    python3 -m pytest tests/test_session.py -q
"""
import argparse
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scenarios"))

import run_task  # noqa: E402
from fake_world import FakeWorld  # noqa: E402

CFG = yaml.safe_load((ROOT / "configs/harness.yaml").read_text(encoding="utf-8"))


class ScriptedLLM:
    def __init__(self, replies):
        self.replies = list(replies)
        self.seen = []

    def chat(self, messages, tools):
        self.seen.append(list(messages))
        return self.replies.pop(0)

    def describe(self):
        return {"name": "scripted", "model": "scripted"}


def call(name, arguments):
    return {"role": "assistant", "content": "", "tool_calls": [
        {"id": "c1", "type": "function", "function": {"name": name, "arguments": json.dumps(arguments)}}]}


def answer(text):
    return {"role": "assistant", "content": text}


def test_second_command_sees_first_and_holding(tmp_path, monkeypatch):
    monkeypatch.setattr(run_task, "ROOT", tmp_path)                  # trace 写到临时目录
    (tmp_path / "traces").mkdir()
    (tmp_path / "experience").mkdir()
    world = FakeWorld({"mouse": [[0.65, -0.20, 0.72]]})
    llm = ScriptedLLM([
        call("find_objects", {"query": "mouse"}),
        call("touch_object", {"object_id": "mouse_1", "arm": "right", "gap_cm": 0}),
        call("gripper", {"arm": "right", "action": "close"}),
        answer("拿起来了"),
        call("get_robot_state", {}),
        answer("手里是鼠标"),
    ])
    args = argparse.Namespace(real=False, confirm=False, plan_only=False)
    session = run_task.Session(llm, world, CFG, args)
    session.run("拿起鼠标")
    session.run("你手里拿着什么")

    second = llm.seen[4]                                              # 第二条指令的第一次提问
    texts = [m.get("content") for m in second]
    assert texts[1:4] == ["拿起鼠标", "拿起来了", "你手里拿着什么"]      # 前一条的问答在前面
    state = json.loads(llm.seen[5][-1]["content"])                    # get_robot_state 的结果
    assert state["holding"]["right"]["object_id"] == "mouse_1"        # 跨指令记得
    assert session.gateway.moves == 0                                 # 每条指令运动次数清零(第二条没动)
