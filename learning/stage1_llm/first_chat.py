import json
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # learning/stage1_llm/xxx.py → 项目根目录
API_URL = "https://api.deepseek.com/chat/completions"
MODEL = "deepseek-flash"


def read_key(env_path, name="DEEPSEEK_API_KEY"):
    """从 .env 文件里找 name=... 这一行,返回等号后面的值;找不到就报错。"""
    for line in env_path.read_text(encoding="utf-8").splitlines():
        if line.startswith(name + "="):
            return line.split("=", 1)[1].strip()
    raise RuntimeError(f"{env_path} 里没有 {name}=")


def chat(key, messages,tools=None):
    """把对话发给 DeepSeek,返回模型的那条回复(一个 dict,含 role 和 content)。"""
    # 请求体:一个普通的 Python dict,字段含义见 DeepSeek 文档的 Chat Completions 接口
    body = {"model": MODEL, "messages": messages, "thinking": {"type": "disabled"}}
    if tools:
        body["tools"] = tools
    request = urllib.request.Request(
        API_URL,
        data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {key}"},
    )

    try:
        # urlopen:连上服务器、发送请求、等待回复
        with urllib.request.urlopen(request, timeout=60) as response:
            data = json.load(response)      # 读出全部内容,并把 JSON 解析成 dict
    except urllib.error.HTTPError as e:
        raise RuntimeError(f"接口返回 {e.code}:{e.read().decode('utf-8', 'replace')}") from e

    return data["choices"][0]["message"]


def main():
    key = read_key(ROOT / ".env")

    messages = [
        {"role": "system", "content": "你是智元 G2 机器人的助手"},
        {"role": "user", "content": "晚上好"},
    ]
    reply = chat(key, messages)
    print("大模型:", reply["content"])


if __name__ == "__main__":
    main()
