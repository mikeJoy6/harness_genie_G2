import json

from first_chat import ROOT, chat, read_key

TOOLS = [
    {
        "type":"function",
        "function":{
            "name": "get_left_end_pose",
            "description":"读取机器人左臂末端在base_link坐标系下的位置,仅读取",
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
]

def main():
    key = read_key(ROOT / ".env")
