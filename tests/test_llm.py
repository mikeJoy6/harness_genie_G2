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
        assert isinstance(llm, ChatCompletionsLLM)
        assert llm.describe()["name"] == name
