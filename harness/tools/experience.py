"""经验工具:read_experience(读一条经验全文)、propose_experience(提议一条新经验,人审核后才生效)。
都不会让机器人运动。经验目录在 SYSTEM 里(harness/loop.py 的 build_system)。"""


def _tool(name, description, properties, required):
    return {"type": "function", "function": {
        "name": name, "description": description,
        "parameters": {"type": "object", "properties": properties, "required": list(required)}}}


def make_experience_tools(names, allow_propose=True):
    """names:现有经验的名字(read_experience 的可选值)。没有经验时不给 read_experience。"""
    tools = []
    if names:
        tools.append(_tool(
            "read_experience",
            "读一条经验的全文:原则、做法、常见坑、怎么确认成功。经验是以前做这类任务总结出来的推荐做法,"
            "做不熟悉的操作前看看系统提示里的经验目录,相关就读;读了也可以按实际情况变通。不会让机器人运动。",
            {"name": {"type": "string", "enum": list(names), "description": "经验目录里的名字"}},
            ["name"]))
    if allow_propose:
        tools.append(_tool(
            "propose_experience",
            "这次任务里学到了值得下次参考的做法或教训(比如某个工具不适合某种任务、某个参数要怎么填)时,"
            "提议一条新经验或改进现有经验(name 和现有的相同就是改进)。只会存成待审稿,要人看过才生效,"
            "这次任务里用不上。只写确实验证过的事,没确认成功的不要写成做法。不会让机器人运动。",
            {"name": {"type": "string", "description": "小写英文加下划线,例如 pour_water"},
             "when": {"type": "string", "description": "一句话:什么时候用这条经验"},
             "content": {"type": "string",
                         "description": "markdown 正文,包含四节:## 原则、## 做法、## 常见坑、## 怎么确认成功"}},
            ["name", "when", "content"]))
    return tools
