"""调用 DeepSeek。和第一阶段 first_chat.py 里的一样,放在这里让第二阶段自成一体。"""
import json
import urllib.error
import urllib.request

import g2_config

API_URL = "https://api.deepseek.com/chat/completions"
MODEL = "deepseek-flash"


def read_key(name="DEEPSEEK_API_KEY"):
    env_path = g2_config.ROOT / ".env"
    for line in env_path.read_text(encoding="utf-8").splitlines():
        if line.startswith(name + "="):
            return line.split("=", 1)[1].strip()
    raise RuntimeError(f"{env_path} 里没有 {name}=")


def chat(key, messages, tools=None):
    body = {"model": MODEL, "messages": messages, "thinking": {"type": "disabled"}}
    if tools:
        body["tools"] = tools
    request = urllib.request.Request(
        API_URL,
        data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {key}"},
    )
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            data = json.load(response)
    except urllib.error.HTTPError as e:
        raise RuntimeError(f"接口返回 {e.code}:{e.read().decode('utf-8', 'replace')}") from e
    return data["choices"][0]["message"]
