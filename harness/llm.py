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

    ENDPOINT = "/chat/completions"

    def __init__(self, name, base_url, model, api_key_env=None, extra_body=None, timeout_s=90):
        self.name = name                      # 配置里的名字,例如 gpt;写进 trace
        self.model = model
        self.url = base_url.rstrip("/") + self.ENDPOINT
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
        """messages / tools / 返回值都是 Chat Completions 格式,loop.py 只认这一种。"""
        body = self.request_body(messages, tools)
        data = self.post(body)
        return self.parse_reply(data)

    def post(self, body):
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
        return data

    def parse_reply(self, data):
        message = data["choices"][0]["message"]
        reply = {"role": "assistant", "content": message.get("content") or ""}
        if message.get("reasoning_content"):          # DeepSeek 思考模式:必须原样传回去,否则下一轮报 400
            reply["reasoning_content"] = message["reasoning_content"]
        if message.get("tool_calls"):
            reply["tool_calls"] = message["tool_calls"]
        return reply


class ResponsesLLM(ChatCompletionsLLM):
    """OpenAI Responses 接口:POST <base_url>/responses。gpt-6.2-astra 这类推理模型要带工具只能走这个。

    loop.py 里的对话一直是 Chat Completions 格式;这里发出去前翻译成 Responses 格式,收到后再翻译回来。
    模型的推理过程(reasoning,加密的)存在回复的 responses_output 里,下一轮原样传回去,模型才接得上思路。
    store=False:OpenAI 不保存这次对话。
    """

    ENDPOINT = "/responses"

    def request_body(self, messages, tools):
        body = {"model": self.model, "input": to_responses_input(messages), "tools": to_responses_tools(tools),
                "store": False, "include": ["reasoning.encrypted_content"]}
        for key in self.extra_body:
            body[key] = self.extra_body[key]
        return body

    def parse_reply(self, data):
        texts = []
        tool_calls = []
        for item in data.get("output", []):
            if item.get("type") == "message":
                for part in item.get("content", []):
                    if part.get("type") == "output_text":
                        texts.append(part.get("text", ""))
            elif item.get("type") == "function_call":
                tool_calls.append({"id": item["call_id"], "type": "function",
                                   "function": {"name": item["name"], "arguments": item.get("arguments") or "{}"}})
        if not texts and not tool_calls:
            raise RuntimeError(f"模型接口({self.name})没有返回文字也没有工具调用:status={data.get('status')},"
                               f"{json.dumps(data.get('incomplete_details') or data.get('error'), ensure_ascii=False)}")
        reply = {"role": "assistant", "content": "".join(texts), "responses_output": data["output"]}
        if tool_calls:
            reply["tool_calls"] = tool_calls
        return reply


def to_responses_tools(tools):
    """Chat Completions 的工具说明 {"type": "function", "function": {...}} → Responses 的(少一层)。"""
    result = []
    for tool in tools:
        fn = tool["function"]
        result.append({"type": "function", "name": fn["name"], "description": fn.get("description", ""),
                       "parameters": fn["parameters"],
                       "strict": False})       # strict 要求每个参数都必填,我们的工具有可选参数
    return result


def to_responses_input(messages):
    """Chat Completions 格式的对话 → Responses 的 input 列表。"""
    items = []
    for message in messages:
        role = message["role"]
        if role == "tool":
            items.append({"type": "function_call_output", "call_id": message["tool_call_id"],
                          "output": message["content"]})
        elif role == "assistant" and "responses_output" in message:
            for item in message["responses_output"]:     # 本接口上一轮的原始输出(含推理),原样传回
                items.append(item)
        elif role == "assistant":
            if message.get("content"):                   # 前几条指令的问答(Session.history,只有文字)
                items.append({"role": "assistant", "content": message["content"]})
            for call in message.get("tool_calls") or []:
                items.append({"type": "function_call", "call_id": call["id"],
                              "name": call["function"]["name"], "arguments": call["function"]["arguments"]})
        else:                                            # system / user
            items.append({"role": role, "content": message["content"]})
    return items


# 接口类型 → 类。以后接格式不同的接口,写一个新类(chat 的输入输出和上面一样),
# 在这里登记,yaml 里那一段写 type: 新类型。
LLM_TYPES = {
    "chat_completions": ChatCompletionsLLM,
    "responses": ResponsesLLM,
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
