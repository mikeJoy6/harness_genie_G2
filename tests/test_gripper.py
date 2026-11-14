"""夹爪 + 等待:gripper / wait。假机器人,不需要 ROS 和 GDK。

    python3 -m pytest tests/test_gripper.py -q
"""
import yaml

from harness.backends.fake import FakeBackend
from harness.gateway import Gateway
from harness.loop import run_agent
from harness.tools.objects import make_tools

MOTION = yaml.safe_load(open("configs/harness.yaml", encoding="utf-8"))["motion"]
ONLY_RIGHT = {"left": "", "right": "omnipicker"}         # 真机现状:只有右手装了


def gateway_for(backend=None, **kwargs):
    backend = backend or FakeBackend()
    return backend, Gateway(backend, MOTION, **kwargs)


class ScriptedLLM:
    def __init__(self, replies):
        self.replies = list(replies)
        self.seen = []

    def chat(self, messages, tools):
        self.seen.append(list(messages))
        return self.replies.pop(0)


def call(name, arguments):
    return {"role": "assistant", "content": "", "tool_calls": [
        {"id": "c1", "type": "function", "function": {"name": name, "arguments": arguments}}]}


def always_no(prompt):
    return False


def no_sleep(seconds):
    no_sleep.calls.append(seconds)


no_sleep.calls = []


def test_open_right_gripper():
    backend, gateway = gateway_for()
    result = gateway.execute("gripper", {"arm": "right", "action": "open"})
    assert result["ok"] and result["executed"]
    assert backend.gripper_calls == [("right", "open")] and backend.gripper_open["right"]


def test_left_gripper_missing_reports_failure():
    backend, gateway = gateway_for(FakeBackend(grippers=ONLY_RIGHT))
    result = gateway.execute("gripper", {"arm": "left", "action": "open"})
    assert not result["ok"] and not result["executed"]
    assert "没有识别到夹爪" in result["error"]


def test_bad_arguments():
    _, gateway = gateway_for()
    assert not gateway.execute("gripper", {"arm": "middle", "action": "open"})["ok"]
    assert not gateway.execute("gripper", {"arm": "right", "action": "squeeze"})["ok"]


def test_plan_only_does_not_move_gripper():
    backend, gateway = gateway_for(plan_only=True)
    result = gateway.execute("gripper", {"arm": "right", "action": "close"})
    assert result["ok"] and not result["executed"] and backend.gripper_calls == []


def test_user_refuses_gripper():
    backend, gateway = gateway_for(confirm=always_no)
    result = gateway.execute("gripper", {"arm": "right", "action": "open"})
    assert not result["ok"] and backend.gripper_calls == []


def test_wait_limits(monkeypatch):
    monkeypatch.setattr("harness.gateway.time.sleep", no_sleep)
    _, gateway = gateway_for()
    assert gateway.execute("wait", {"seconds": 5})["waited_s"] == 5.0
    assert not gateway.execute("wait", {"seconds": 0})["ok"]
    assert not gateway.execute("wait", {"seconds": MOTION["max_wait_s"] + 1})["ok"]
    assert not gateway.execute("wait", {"seconds": "5"})["ok"]


def test_robot_state_includes_grippers():
    _, gateway = gateway_for(FakeBackend(grippers=ONLY_RIGHT))
    state = gateway.execute("get_robot_state", {})
    assert state["grippers"]["right"]["model"] == "omnipicker" and state["grippers"]["left"]["model"] == ""


def test_agent_open_wait_close(monkeypatch):
    """“你先打开夹爪过5秒再闭合” → gripper(open) → wait(5) → gripper(close)。"""
    no_sleep.calls = []
    monkeypatch.setattr("harness.gateway.time.sleep", no_sleep)
    backend, gateway = gateway_for()
    llm = ScriptedLLM([call("gripper", '{"arm": "right", "action": "open"}'),
                       call("wait", '{"seconds": 5}'),
                       call("gripper", '{"arm": "right", "action": "close"}'),
                       {"role": "assistant", "content": "完成"}])
    assert run_agent("你先打开右手夹爪过5秒再闭合", llm, gateway, make_tools()) == "完成"
    assert backend.gripper_calls == [("right", "open"), ("right", "close")]
    assert no_sleep.calls == [5.0]


def test_agent_sees_left_failure():
    """左手没装:工具返回失败,模型下一轮能看到 error。"""
    backend, gateway = gateway_for(FakeBackend(grippers=ONLY_RIGHT))
    llm = ScriptedLLM([call("gripper", '{"arm": "left", "action": "open"}'),
                       {"role": "assistant", "content": "左手夹爪打不开"}])
    run_agent("打开左手夹爪", llm, gateway, make_tools())
    last_tool_message = llm.seen[-1][-1]
    assert last_tool_message["role"] == "tool" and "没有识别到夹爪" in last_tool_message["content"]


def test_tools_listed():
    names = [t["function"]["name"] for t in make_tools()]
    assert "gripper" in names and "wait" in names
