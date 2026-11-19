"""调用大模型。只用标准库,不需要额外安装 openai 包。

用哪个模型在 configs/harness.yaml 的 llms 段里配,每个模型一段,起一个名字;
run_task.py / run_scenarios.py 用 --llm 名字 选,不填就用 default_llm。
OpenAI、DeepSeek、通义、Kimi、本地 vLLM / Ollama 等都是同一种接口(Chat Completions),
加一种模型只要在 yaml 里加一段,不用改代码。
"""
import json
import os
import urllib.error
import urllib.request


class ChatCompletionsLLM:
    """OpenAI Chat Completions 格式的接口:POST <base_url>/chat/completions。"""

    def __init__(self, name, base_url, model, api_key_env=None, extra_body=None, timeout_s=90):
        self.name = name                      # 配置里的名字,例如 gpt;写进 trace
        self.model = model
        self.url = base_url.rstrip("/") + "/chat/completions"
        self.extra_body = extra_body or {}    # 这家特有的请求字段,例如 DeepSeek 的 thinking
        self.timeout_s = timeout_s
        self.api_key = None
        if api_key_env:                       # 本地模型可以不要 key
            self.api_key = os.environ.get(api_key_env)
            if not self.api_key:
                raise RuntimeError(f"环境变量 {api_key_env} 没有设置(填在 .env 里)")
            if not self.api_key.isascii():
                raise RuntimeError(f"{api_key_env} 里有中文或全角字符,请填写真实的 API key")

    def describe(self):
        """写进 trace 第一行,以后看 trace 知道是哪个模型跑的。"""
        return {"name": self.name, "model": self.model, "url": self.url}

    def request_body(self, messages, tools):
        body = {"model": self.model, "messages": messages, "tools": tools}
        for key in self.extra_body:
            body[key] = self.extra_body[key]
        return body

    def chat(self, messages, tools):
        body = self.request_body(messages, tools)
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        request = urllib.request.Request(
            self.url, data=json.dumps(body, ensure_ascii=False).encode("utf-8"), method="POST", headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                data = json.load(response)
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"模型接口({self.name})返回 {e.code}:{e.read().decode('utf-8', 'replace')[:500]}") from e

        message = data["choices"][0]["message"]
        reply = {"role": "assistant", "content": message.get("content") or ""}
        if message.get("reasoning_content"):          # DeepSeek 思考模式:必须原样传回去,否则下一轮报 400
            reply["reasoning_content"] = message["reasoning_content"]
        if message.get("tool_calls"):
            reply["tool_calls"] = message["tool_calls"]
        return reply


# 接口类型 → 类。以后接格式不同的接口(不是 Chat Completions),写一个新类(chat 的输入输出和上面一样),
# 在这里登记,yaml 里那一段写 type: 新类型。
LLM_TYPES = {
    "chat_completions": ChatCompletionsLLM,
}


def make_llm(cfg, name=None):
    """cfg:整个 harness.yaml;name:llms 里的名字,None = default_llm。"""
    if name is None:
        name = cfg["default_llm"]
    profiles = cfg["llms"]
    if name not in profiles:
        raise RuntimeError(f"configs/harness.yaml 的 llms 里没有 '{name}',可选:{list(profiles)}")
    settings = dict(profiles[name])
    kind = settings.pop("type", "chat_completions")
    if kind not in LLM_TYPES:
        raise RuntimeError(f"llms.{name}.type = '{kind}' 不认识,可选:{list(LLM_TYPES)}")
    return LLM_TYPES[kind](name, **settings)
