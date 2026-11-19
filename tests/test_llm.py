"""harness/llm.py:按名字选模型、extra_body、key 检查。不联网(不调用 chat)。

    python3 -m pytest tests/test_llm.py -q
"""
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness.llm import ChatCompletionsLLM, make_llm  # noqa: E402

CFG = {
    "default_llm": "a",
    "llms": {
        "a": {"base_url": "https://a.example/v1/", "model": "model-a", "api_key_env": "TEST_KEY_A",
              "extra_body": {"thinking": {"type": "disabled"}}},
        "b": {"base_url": "http://127.0.0.1:8000/v1", "model": "model-b", "api_key_env": None},
    },
}


def test_default_and_by_name(monkeypatch):
    monkeypatch.setenv("TEST_KEY_A", "sk-test")
    a = make_llm(CFG)
    assert a.name == "a" and a.model == "model-a"
    assert a.url == "https://a.example/v1/chat/completions"
    b = make_llm(CFG, "b")
    assert b.name == "b" and b.api_key is None          # 本地模型不要 key


def test_extra_body_is_merged(monkeypatch):
    monkeypatch.setenv("TEST_KEY_A", "sk-test")
    body = make_llm(CFG, "a").request_body([{"role": "user", "content": "hi"}], [])
    assert body["model"] == "model-a"
    assert body["thinking"] == {"type": "disabled"}
    body_b = make_llm(CFG, "b").request_body([], [])
    assert "thinking" not in body_b


def test_unknown_name_lists_choices():
    with pytest.raises(RuntimeError) as e:
        make_llm(CFG, "nope")
    assert "'a'" in str(e.value) and "'b'" in str(e.value)


def test_unknown_type():
    cfg = {"default_llm": "x", "llms": {"x": {"type": "nope", "base_url": "u", "model": "m"}}}
    with pytest.raises(RuntimeError):
        make_llm(cfg)


def test_missing_key(monkeypatch):
    monkeypatch.delenv("TEST_KEY_A", raising=False)
    with pytest.raises(RuntimeError):
        make_llm(CFG, "a")


def test_real_config_profiles_build(monkeypatch):
    """configs/harness.yaml 里每一段都能建出来(key 用假的)。"""
    cfg = yaml.safe_load((ROOT / "configs/harness.yaml").read_text(encoding="utf-8"))
    assert cfg["default_llm"] in cfg["llms"]
    for name, profile in cfg["llms"].items():
        if profile.get("api_key_env"):
            monkeypatch.setenv(profile["api_key_env"], "sk-test")
        llm = make_llm(cfg, name)
        assert isinstance(llm, ChatCompletionsLLM)      # ResponsesLLM 是它的子类
        assert llm.describe()["name"] == name


# ---- Responses 接口(gpt-6.2-astra 用):格式翻译,不联网 ----

def responses_llm(monkeypatch):
    monkeypatch.setenv("TEST_KEY_A", "sk-test")
    cfg = {"default_llm": "r", "llms": {"r": {"type": "responses", "base_url": "https://r.example/v1",
                                              "model": "m", "api_key_env": "TEST_KEY_A",
                                              "extra_body": {"reasoning": {"effort": "low"}}}}}
    return make_llm(cfg)


def test_responses_request(monkeypatch):
    llm = responses_llm(monkeypatch)
    assert llm.url == "https://r.example/v1/responses"
    tools = [{"type": "function", "function": {"name": "find_objects", "description": "找",
                                               "parameters": {"type": "object", "properties": {}}}}]
    messages = [
        {"role": "system", "content": "S"},
        {"role": "user", "content": "拿起鼠标"},
        {"role": "assistant", "content": "", "responses_output": [{"type": "reasoning", "id": "rs_1"},
                                                                  {"type": "function_call", "call_id": "c1",
                                                                   "name": "find_objects", "arguments": "{}"}],
         "tool_calls": [{"id": "c1", "type": "function", "function": {"name": "find_objects", "arguments": "{}"}}]},
        {"role": "tool", "tool_call_id": "c1", "content": "{\"ok\": true}"},
    ]
    body = llm.request_body(messages, tools)
    assert body["reasoning"] == {"effort": "low"} and body["store"] is False
    assert body["tools"] == [{"type": "function", "name": "find_objects", "description": "找",
                              "parameters": {"type": "object", "properties": {}}, "strict": False}]
    items = body["input"]
    assert items[0] == {"role": "system", "content": "S"}
    assert items[2] == {"type": "reasoning", "id": "rs_1"}            # 推理原样传回
    assert items[3]["type"] == "function_call" and items[3]["call_id"] == "c1"
    assert items[4] == {"type": "function_call_output", "call_id": "c1", "output": "{\"ok\": true}"}


def test_responses_history_text_only(monkeypatch):
    """Session.history 里只有文字的 assistant(可能是别的模型写的)也能翻译。"""
    body = responses_llm(monkeypatch).request_body([{"role": "assistant", "content": "拿好了"}], [])
    assert body["input"] == [{"role": "assistant", "content": "拿好了"}]


def test_responses_reply(monkeypatch):
    llm = responses_llm(monkeypatch)
    output = [{"type": "reasoning", "id": "rs_1", "summary": []},
              {"type": "message", "role": "assistant", "content": [{"type": "output_text", "text": "先找"}]},
              {"type": "function_call", "id": "fc_1", "call_id": "c9", "name": "find_objects",
               "arguments": "{\"query\": \"mouse\"}"}]
    reply = llm.parse_reply({"status": "completed", "output": output})
    assert reply["content"] == "先找"
    assert reply["tool_calls"] == [{"id": "c9", "type": "function",
                                    "function": {"name": "find_objects", "arguments": "{\"query\": \"mouse\"}"}}]
    assert reply["responses_output"] == output


def test_responses_empty_reply_raises(monkeypatch):
    llm = responses_llm(monkeypatch)
    with pytest.raises(RuntimeError):
        llm.parse_reply({"status": "incomplete", "output": [{"type": "reasoning", "id": "rs_1"}],
                         "incomplete_details": {"reason": "max_output_tokens"}})
