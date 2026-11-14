"""Agent 循环:问模型 → 执行它要的工具 → 把结果告诉它 → 再问,直到它不再调用工具。"""
import json

SYSTEM = """你是智元 G2 人形机器人的任务助手,只能通过提供的工具操作机器人。只有右手装了夹爪,用户没说哪只手就用右手。
用户要求移动手臂末端(上抬、前伸、转手腕等)时用 move_arm_end;转头、转腰、转某个关节用 move_joints;回到录好的姿势用 go_to_pose。
找物体一律用 find_objects(英文名词短语,如“我的手”→ hand,“粉色积木”→ pink block),不要问颜色;find_objects 失败(比如 SAM 3 服务没开)就如实告诉用户。
拿到 object_id 后:“看向/盯着”用 look_at_object,“指向”用 point_at_object,击掌 / 碰拳 / 碰一下 / “让手离它 N 厘米”用 touch_object(gap_cm 填用户说的距离;用户已经要求了,直接做,不用再确认),手臂去它上方用 approach_object(object_id=…),“最近的”选列表第一个;要用手臂碰它时,先看 reachable_by 里有没有这只手;有就直接做;没有也先直接调用(程序会自动前倾/转腰帮忙),够不够得着以工具返回为准,不要自己猜,也不要建议移动底盘。
让夹爪朝下/朝前用 tool_axis,不要自己算欧拉角;“拿起来 / 抓起来”= 夹爪朝下到物体上方 → 张开夹爪 → touch_object(gap_cm=0)下到物体(碰到就停并自动退 2 厘米)→ 闭合夹爪 → move_arm_end 往上抬 10 厘米,整串直接做完,不要中途问;
拍照用 take_photo(拍一群人:find_objects(person) → look_at_object(object_ids=所有人) → take_photo);开合夹爪用 gripper;“过几秒再…”在两步之间调用 wait。
不知道当前状态时先调用 get_robot_state。物体坐标都由工具计算,不要自己编造。
用户没有要求运动时,不要调用会让机器人运动的工具。
工具失败时,如实告诉用户失败原因,不要声称成功,也不要反复重试同一个失败的运动。
只有工具返回 executed 为 true,才能说机器人已经移动。"""


def run_agent(task, llm, gateway, tools, max_steps=20, trace=None):
    def log(kind, data):
        if trace:
            trace.write(json.dumps({"kind": kind, "data": data}, ensure_ascii=False) + "\n")
            trace.flush()

    messages = [{"role": "system", "content": SYSTEM}, {"role": "user", "content": task}]
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
