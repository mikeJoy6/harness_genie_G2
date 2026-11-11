import json
import readline  # 让 input() 正确编辑中文，并支持方向键历史记录
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


def chat(key, messages, tools=None):
    """把对话发给 DeepSeek,返回模型的那条回复(一个 dict,含 role 和 content)。"""
    body = {"model": MODEL, "messages": messages, "thinking": {"type": "disabled"}}
    if tools:
        body["tools"] = tools
    request = urllib.request.Request(
        API_URL,
        data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {key}"},
    )
    try:
        print("请求:", body)
        with urllib.request.urlopen(request, timeout=60) as response:
            data = json.load(response)
    except urllib.error.HTTPError as e:
        raise RuntimeError(f"接口返回 {e.code}:{e.read().decode('utf-8', 'replace')}") from e
    return data["choices"][0]["message"]


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
            "description": "让左臂末端沿直线(笛卡尔路径)移动到 base_link 坐标系下的目标点,姿态保持不变。会让机器人运动。"
                           "单次移动不超过 0.30 米;目标超出工作空间或距离太远会被拒绝,并返回原因。",
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

# ---------------- gateway 的安全参数 ----------------
#   WORKSPACE:左手允许到达的范围,每个轴一个 (最小值, 最大值),单位米
#   MAX_TRAVEL_M:单次移动最远多少米,先用 0.30
WORKSPACE = {"x":(0.00,0.90),"y":(-0.50,0.50),"z":(0.00,2.00)} 
MAX_TRAVEL_M = 0.3


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

# 会让机器人运动的工具。只有它们需要过 gateway,只读的工具直接执行
MOTION_TOOLS = {"move_left_end_linear"}


def check_move(x, y, z):
    """gateway:检查一次移动安不安全。
    安全 → 返回 None;不安全 → 返回一句原因(字符串),这句话会交给模型看。
    """

    # 检查 1:工作空间。每个轴都要在 (最小, 最大) 之间
    for axis, value in {"x": x, "y": y, "z": z}.items():
        low, high = WORKSPACE[axis]
        if not low <= value <= high:
            return f"{axis}={value} 超出工作空间 {low}~{high} 米"

    # 检查 2:单次移动距离。复用 compute_offset,不重写公式
    distance = compute_offset(left_end_xyz, [x, y, z])["distance_m"]
    if distance > MAX_TRAVEL_M:
        return f"这次要移动 {distance} 米,超过单次上限 {MAX_TRAVEL_M} 米,请分几次移动"

    return None


def ask_human(tool_name, args):
    """真机上动之前要人确认。输入 y 才返回 True。"""
    print(f"\n[gateway] 准备执行 {tool_name} {args}")
    print(f"[gateway] 当前左手位置 {left_end_xyz}")
    answer = input("执行吗?[y/N] ").strip().lower()
    return answer == "y"                     # 只有 y 算同意,直接回车也是拒绝


def run_tool(tool_name, raw_args):
    """执行一个工具,返回结果 dict。任何错误都变成 ok=False 交给模型,程序不崩。"""
    func = TOOL_FUNCS.get(tool_name)
    if func is None:
        return {"ok": False, "error": f"没有这个工具:{tool_name}"}
    try:
        args = json.loads(raw_args)          # 参数是 JSON 字符串,先转成 dict

        # gateway:只拦会运动的工具。被拒绝就 return,后面的 func(**args) 不执行,机器人不动
        if tool_name in MOTION_TOOLS:
            reason = check_move(**args)                  # 先让代码检查
            if reason is not None:
                print(f"[gateway] 拒绝:{reason}")
                return {"ok": False, "error": reason}
            if not ask_human(tool_name, args):           # 代码检查过了,再问人
                return {"ok": False, "error": "操作员拒绝执行"}

        return func(**args)                  # {"color": "红色"} → locate_object(color="红色")
    except Exception as e:                   # 参数格式错、参数名不对等
        return {"ok": False, "error": f"{type(e).__name__}: {e}"}


def run_agent(key, task, max_steps=8):
    """agent loop:问模型 → 执行它要的工具 → 把结果交还 → 再问,直到它不再要工具。"""
    messages = [
        {"role": "system", "content": SYSTEM},
        {"role": "user", "content": task},
    ]

    for step in range(max_steps):
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

