"""Agent 循环:问模型 → 执行它要的工具 → 把结果告诉它 → 再问,直到它不再调用工具。"""
import json

SYSTEM_BASE = """# 你是谁
智元 G2 人形机器人的任务助手,只能通过提供的工具了解和操作机器人。用户用什么语言,你就用什么语言回答。

# 机器人的事实
- 只有右手装了夹爪;用户没说哪只手就用右手。
- 没有底盘,不能走。手臂从肩膀算起大约能伸 0.8 米,够不着时程序会自动前倾 / 转腰帮忙;够不够得着以工具返回为准,不要自己猜,也不要建议移动底盘。
- 物体只能用 find_objects 找(英文名词),之后用它给的编号指代;坐标都由工具算,不要自己编。
- get_robot_state 里的 holding 是程序按夹爪动作推测的“手里拿着什么”,没有验证过。手里的东西不在碰撞检查里。
- 夹爪不回报开合位置:gripper 返回 ok 只说明命令发出去了。

# 必须遵守
- 用户没要求运动,就不调用会让机器人运动的工具;用户要求了就直接做完,不用再确认。
- 只有工具返回 executed 为 true,才能说机器人动了。说“完成”之前,能用 find_objects 等工具确认结果的就确认;确认不了就照实说哪一步没确认。
- 工具失败先看失败原因:是参数的问题就调整参数再试;是做不到就停下,把原因告诉用户。不要换一个意思不同的工具去凑(比如拿“碰”的工具去“放”东西)。
- 接触人只在用户要求时(击掌、碰拳、递东西给他)。用户没要求接触时,不要让夹爪或手里的东西靠近人。

# 工作方法
- 前面几条指令的问答会附在这条前面,可以参考;但机器人现在的状态以 get_robot_state 为准(之前说的可能已经变了)。
- 多步操作先想好步骤和每一步的预期结果;每次工具返回后对照检查,不符合预期就停下想原因。
- 运动次数有上限(get_robot_state 的 moves_left),长任务先算一下够不够。
- 做不熟悉的操作前,看看下面的经验目录,相关就用 read_experience 读全文;读了也可以按实际情况变通。
- 任务里学到值得下次参考的做法或教训,可以用 propose_experience 提议(人审核后才生效)。"""


def build_system(catalog_text=""):
    """SYSTEM = 固定部分 + 经验目录(ExperienceBook.catalog_text,每条一行)。"""
    if not catalog_text:
        return SYSTEM_BASE
    return SYSTEM_BASE + "\n\n# 经验目录\n" + catalog_text


SYSTEM = build_system()


def run_agent(task, llm, gateway, tools, max_steps=20, trace=None, system=None, history=None):
    """system:系统提示,默认 SYSTEM(没有经验目录);history:前几条指令的对话 [{"role", "content"}, ...],
    放在这条指令前面,模型才知道刚才做过什么。"""
    def log(kind, data):
        if trace:
            trace.write(json.dumps({"kind": kind, "data": data}, ensure_ascii=False) + "\n")
            trace.flush()

    messages = [{"role": "system", "content": system or SYSTEM}]
    for turn in history or []:
        messages.append(turn)
    messages.append({"role": "user", "content": task})
    log("user", task)

    for _ in range(max_steps):
        reply = llm.chat(messages, tools)
        messages.append(reply)
        log("assistant", reply)

        calls = reply.get("tool_calls") or []
        if not calls:
            return reply.get("content", "")

        for call in calls:
            name = call["function"]["name"]
            print(f"  → 模型调用 {name} {call['function']['arguments']}", flush=True)
            result = _execute(call, gateway)
            print(f"  ← {json.dumps(result, ensure_ascii=False)}", flush=True)
            log("tool", {"name": name, "result": result})
            messages.append({"role": "tool", "tool_call_id": call["id"],
                             "content": json.dumps(result, ensure_ascii=False)})

    return f"超过最大步数 {max_steps},已停止。"


def _execute(call, gateway):
    fn = call["function"]
    try:
        args = json.loads(fn["arguments"] or "{}")
    except json.JSONDecodeError as e:
        return {"ok": False, "error": f"参数不是合法 JSON:{e}"}
    if not isinstance(args, dict):
        return {"ok": False, "error": "参数必须是 JSON 对象"}
    return gateway.execute(fn["name"], args)
