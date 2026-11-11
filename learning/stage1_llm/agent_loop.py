import json
import readline  # 让 input() 正确编辑中文，并支持方向键历史记录

from first_chat import ROOT, chat, read_key

TOOLS = [
    {
        "type":"function",
        "function":{
            "name": "get_left_end_pose",
            "description": "读取机器人左臂末端在 base_link 坐标系下的位置,单位米。只读取,不会让机器人运动。",
            # 不需要参数:properties 留空,required 也留空
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },

    {
        "type":"function",
        "function":{
            "name": "locate_object",
            "description":"按颜色在桌面上找物体,返回物体中心在 base_link 坐标系下的位置,单位米。找不到时返回 ok=false。只识别,不会让机器人运动",
            "parameters": {
                "type": "object",
                "properties": {
                    "color":{
                        "type": "string",
                        "enum": ["红色", "蓝色", "绿色"],
                        "description": "物品的颜色",
                    },
                },
                "required": ["color"],
            },
        },
    },

    {
        "type": "function",
        "function": {
            "name": "move_left_end_linear",
            "description": "让左臂末端沿直线(笛卡尔路径)移动到 base_link 坐标系下的目标点,姿态保持不变。会让机器人运动。",
            # 三个必填的数字参数
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "目标点 x,单位米,向前为正"},
                    "y": {"type": "number", "description": "目标点 y,单位米,向左为正"},
                    "z": {"type": "number", "description": "目标点 z,单位米,向上为正"},
                },
                "required": ["x", "y", "z"],
            },
        },
    },

    {
        "type": "function",
        "function": {
            "name": "compute_offset",
            "description": "计算从起点到终点的偏差:返回 dx、dy、dz 和直线距离,单位米。只做计算,不会让机器人运动。",
            # 参数是两个数组,每个数组正好 3 个数字
            "parameters": {
                "type": "object",
                "properties": {
                    "from_xyz": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "起点 [x, y, z],单位米",
                    },
                    "to_xyz": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "终点 [x, y, z],单位米",
                    },
                },
                "required": ["from_xyz", "to_xyz"],
            },
        },
    },
]

# 假机器人的状态:左臂末端当前位置(米)。移动工具会修改它,读取工具会读它
left_end_xyz = [0.45, 0.20, 0.95]


def get_left_end_pose():
    return {"frame": "base_link", "xyz_m": list(left_end_xyz)}


def move_left_end_linear(x, y, z):
    """假的笛卡尔直线运动:直接把位置改成目标点。"""
    start = list(left_end_xyz)
    left_end_xyz[:] = [x, y, z]      # [:] 表示原地改列表里的值,不是新建一个变量
    return {"ok": True, "from_xyz_m": start, "to_xyz_m": [x, y, z]}


def compute_offset(from_xyz, to_xyz):
    d = [round(b - a, 4) for a, b in zip(from_xyz, to_xyz)]
    distance = round((d[0] ** 2 + d[1] ** 2 + d[2] ** 2) ** 0.5, 4)
    return {"dx_m": d[0], "dy_m": d[1], "dz_m": d[2], "distance_m": distance}

def locate_object(color):
    """假的物体识别:只认识红色。"""
    if color == "红色":
        return {"ok": True, "frame": "base_link", "xyz_m": [0.60, 0.25, 0.80]}
    if color == "蓝色":
            return {"ok": True, "frame": "base_link", "xyz_m": [0.60, 0.25, 0.80]}
    if color == "绿色":
            return {"ok": True, "frame": "base_link", "xyz_m": [0.60, 0.25, 0.80]}
    return {"ok": False, "error": f"没看到{color}的物体"}


# 工具名(模型说的字符串) → 函数(你的代码)。必须写在所有函数定义之后
TOOL_FUNCS = {
    "get_left_end_pose": get_left_end_pose,
    "locate_object": locate_object,
    "move_left_end_linear": move_left_end_linear,
    "compute_offset": compute_offset,
}

SYSTEM = "你是智元 G2 机器人的助手,只能通过工具了解和控制机器人。没有合适的工具时,直接说明做不到。"


def run_tool(tool_name, raw_args):
    """执行一个工具,返回结果 dict。任何错误都变成 ok=False 交给模型,程序不崩。"""
    func = TOOL_FUNCS.get(tool_name)
    if func is None:
        return {"ok": False, "error": f"没有这个工具:{tool_name}"}
    try:
        args = json.loads(raw_args)          # 参数是 JSON 字符串,先转成 dict
        return func(**args)                  # {"color": "红色"} → locate_object(color="红色")
    except Exception as e:                   # 参数格式错、参数名不对等
        return {"ok": False, "error": f"{type(e).__name__}: {e}"}


def run_agent(key, task, max_steps=6):
    """agent loop:问模型 → 执行它要的工具 → 把结果交还 → 再问,直到它不再要工具。"""
    messages = [
        {"role": "system", "content": SYSTEM},
        {"role": "user", "content": task},
    ]

    for step in range(max_steps):
        print("请求:", messages)
        reply = chat(key, messages, TOOLS)
        messages.append(reply)               # 模型这条消息(含 tool_calls)必须放回对话

        if not reply.get("tool_calls"):      # 不再要工具 → 这就是最终回答
            return reply["content"]

        for call in reply["tool_calls"]:     # 一轮里可能要好几个工具
            tool_name = call["function"]["name"]
            raw_args = call["function"]["arguments"]
            result = run_tool(tool_name, raw_args)
            print(f"[第{step}步] {tool_name}({raw_args}) -> {result}")

            messages.append({                # 结果作为 tool 消息交还,用 id 对上号
                "role": "tool",
                "tool_call_id": call["id"],
                "content": json.dumps(result, ensure_ascii=False),
            })

    return f"已经用完 {max_steps} 步,任务没有完成"


def main():
    key = read_key(ROOT / ".env")
    print("可以输入语言指令；输入 q 退出。")

    while True:
        try:
            task = input("你: ").strip()
        except EOFError:                      # Ctrl+D
            print("\n退出。")
            return

        if task.lower() in ("q", "quit", "exit"):
            print("退出。")
            return
        if not task:                          # 空行不发给模型
            continue

        answer = run_agent(key, task)
        print("大模型:", answer)


if __name__ == "__main__":
    main()

