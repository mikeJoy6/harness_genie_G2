"""给大模型用的工具:说明书(TOOLS) + 真正执行的函数(make_handlers)。

和第一阶段的区别:函数不再返回假数据,而是通过 RosIO 读真实的 ROS 数据。
三个工具全部只读,所以这里不需要 gateway。
"""
import math

import numpy as np

import config

TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "get_joint_angles",
            "description": "读取机器人某个部位的当前关节角,单位度,按关节 1、2、3... 的顺序。只读取,不会让机器人运动。",
            "parameters": {
                "type": "object",
                "properties": {
                    "group": {"type": "string", "enum": list(config.JOINT_GROUPS),
                              "description": "waist 腰,head 头,left_arm 左臂,right_arm 右臂"},
                },
                "required": ["group"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_link_position",
            "description": "读取机器人某个部件(link)在 base_link 坐标系下的位置,单位米。"
                           "x 向前、y 向左、z 向上。只读取,不会让机器人运动。",
            "parameters": {
                "type": "object",
                "properties": {
                    "link": {"type": "string", "enum": config.LINKS_FOR_LLM,
                             "description": "arm_l_end_link 左手末端,arm_r_end_link 右手末端,head_link3 头,"
                                            "body_link5 腰顶,arm_l_link4/arm_r_link4 左/右肘"},
                },
                "required": ["link"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "check_joint_change",
            "description": "假设把某条手臂的某个关节在当前角度上再转一些,问 MoveIt 整台机器人会不会自碰撞。"
                           "只是检查,机器人不会运动。",
            "parameters": {
                "type": "object",
                "properties": {
                    "group": {"type": "string", "enum": ["left_arm", "right_arm"]},
                    "joint_index": {"type": "integer", "minimum": 1, "maximum": 7,
                                    "description": "第几个关节,1~7,从肩到腕"},
                    "delta_deg": {"type": "number", "description": "在当前角度上再转多少度,可正可负"},
                },
                "required": ["group", "joint_index", "delta_deg"],
            },
        },
    },
]


def make_handlers(ros):
    """返回 {工具名: 函数}。函数里用到的 ros 是传进来的,所以这里写成"函数里定义函数"。"""

    def get_joint_angles(group):
        values = ros.joints(config.JOINT_GROUPS[group])
        return {"ok": True, "group": group, "degrees": [round(math.degrees(v), 2) for v in values.values()]}

    def get_link_position(link):
        xyz, _ = ros.lookup(config.BASE_FRAME, link)
        return {"ok": True, "link": link, "frame": config.BASE_FRAME, "xyz_m": np.round(xyz, 4).tolist()}

    def check_joint_change(group, joint_index, delta_deg):
        name = config.JOINT_GROUPS[group][joint_index - 1]
        new_value = ros.joints([name])[name] + math.radians(delta_deg)
        res = ros.check_collision({name: new_value})
        if not res["available"] or res["valid"] is None:
            return {"ok": False, "error": res["error"]}
        return {"ok": True, "joint": name, "collision": not res["valid"],
                "contacts": [f"{a} 碰 {b}" for a, b in res["contacts"]]}

    return {
        "get_joint_angles": get_joint_angles,
        "get_link_position": get_link_position,
        "check_joint_change": check_joint_change,
    }
