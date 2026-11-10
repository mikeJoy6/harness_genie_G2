"""调用大模型。只用标准库,不需要额外安装 openai 包。"""
import json
import os
import urllib.error
import urllib.request


class DeepSeekLLM:
    def __init__(self, base_url, model, api_key_env, thinking="disabled", timeout_s=90):
        self.api_key = os.environ.get(api_key_env)
        if not self.api_key:
            raise RuntimeError(f"环境变量 {api_key_env} 没有设置")
        if not self.api_key.isascii():
            raise RuntimeError(f"{api_key_env} 里有中文或全角字符,请换成 DeepSeek 控制台里真实的 key")
        self.url = base_url.rstrip("/") + "/chat/completions"
        self.model = model
        self.thinking = thinking
        self.timeout_s = timeout_s

    def chat(self, messages, tools):
        body = {"model": self.model, "messages": messages, "tools": tools,
                "thinking": {"type": self.thinking}}
        request = urllib.request.Request(
            self.url, data=json.dumps(body, ensure_ascii=False).encode("utf-8"), method="POST",
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {self.api_key}"})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                data = json.load(response)
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"模型接口返回 {e.code}:{e.read().decode('utf-8', 'replace')[:500]}") from e

        message = data["choices"][0]["message"]
        reply = {"role": "assistant", "content": message.get("content") or ""}
        if message.get("reasoning_content"):          # 思考模式下必须原样传回去,否则下一轮报 400
            reply["reasoning_content"] = message["reasoning_content"]
        if message.get("tool_calls"):
            reply["tool_calls"] = message["tool_calls"]
        return reply
