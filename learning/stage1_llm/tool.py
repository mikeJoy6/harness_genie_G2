import json

from first_chat import ROOT, chat, read_key

TOOLS = [
    {
        "type":"function",
        "function":{
            "name": "get_left_end_pose",
            "description":"读取机器人左臂末端在base_link坐标系下的位置,仅读取",
            "parameters": {
                "type": "object",
                "properties": {
                    "unit": {
                        "type": "string",
                        "enum": ["m", "cm", "mm"],
                        "description": "坐标的单位:m 米,cm 厘米,mm 毫米",
                    },
                },
                "required": ["unit"],
            },
        },
    },
]

def get_left_end_pose(unit):
    scale = {"m": 1, "cm": 100, "mm": 1000}[unit]
    xyz = [round(v * scale, 3) for v in [0.45, 0.20, 0.95]]   # 假坐标,单位是米
    return {"frame": "base_link", "unit": unit, "xyz": xyz}


def main():
    key = read_key(ROOT / ".env")
    messages = [
        {"role": "system", "content": "你是智元 G2 机器人的助手,只能通过工具了解机器人状态"},
        {"role": "user", "content": "左手现在在哪?用毫米告诉我"},
    ]
    reply = chat(key, messages,TOOLS)
    print(json.dumps(reply, ensure_ascii=False, indent=2))
    """
    输入量是chat() 把下面这个 body 发给 DeepSeek
    {
        "model": "deepseek-flash",
        "messages": [
            {"role": "system", "content": "你是智元 G2 机器人的助手,只能通过工具了解机器人状态"},
            {"role": "user", "content": "告诉我左手的空间坐标"}
        ],
        "thinking": {"type": "disabled"},
        "tools": [
            {
                "type": "function",
                "function": {
                    "name": "get_left_end_pose",
                    "description": "读取机器人左臂末端在base_link坐标系下的位置,仅读取",
                    "parameters": {"type": "object", "properties": {}, "required": []}
                }
            }
        ]
    }

    此时reply长这样:
    {
        "role": "assistant",
        "content": "",
        "tool_calls": [
            {
                "index": 0,
                "id": "call_00_xxxxxxxxxxxx",
                "type": "function",
                "function": {
                    "name": "get_left_end_pose",
                    "arguments": "{}"
                }
            }
        ]
    }
    """

    # 然后接下来让tool_calls真的去调用函数
    for call in reply["tool_calls"]:
        tool_name = call["function"]["name"]
        raw_args = call["function"]["arguments"]
        print("使用工具", tool_name, "参数", raw_args)

        # 参数不能是str格式,想用要转为dict
        args = json.loads(raw_args)

        result = get_left_end_pose(args["unit"])
        print("函数返回:", result)





if __name__ == "__main__":
    main()

