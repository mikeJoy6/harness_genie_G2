"""给模型看的运动工具说明。工具名必须和 harness/gateway.py 里的一致。

模型只能给相对偏移、关节增量/目标角、录好的位姿名;所有上限检查在 gateway 里。
"""
from ..motion.joints import GROUPS

# 关节含义来自 URDF(G2_t2_crsB):零位时各关节轴和 base_link 对齐,x 向前、y 向左、z 向上
GROUP_TEXT = ("body 腰(5 个关节):1、2、3 都绕 y 轴俯仰,三者之和是上身前倾角(正 = 前倾),1 和 2 主要改变腰的高度,"
              "前倾优先改 3;4 绕 x 轴侧倾;5 绕 z 轴转腰(正 = 向左转)。"
              "head 头(3 个):1 绕 z 轴左右转头(正 = 向左,限 ±45°);2 绕 x 轴歪头(限 ±8°);"
              "3 绕 y 轴点头(正 = 低头,限 -1°~30°)。"
              "left_arm / right_arm 左臂 / 右臂(各 7 个,从肩到腕)。")


def _tool(name, description, properties=None, required=()):
    return {"type": "function", "function": {
        "name": name, "description": description,
        "parameters": {"type": "object", "properties": properties or {}, "required": list(required)}}}


def _xyz(description):
    return {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3, "description": description}


def make_motion_tools(pose_names=(), pose_notes=None):
    """pose_notes:{位姿名: 说明},来自 poses.yaml 的 note,写进 go_to_pose 的说明里,
    模型才知道“初始位姿”对应哪个名字。"""
    tools = [
        _tool("get_robot_state",
              "读取腰、头、双臂所有关节的当前角度(度,按关节 1、2、3... 的顺序),"
              "左右臂末端在 base_link 下的位置(米)、姿态 rpy(度)、工具轴方向 tool_axis 和"
              "离竖直朝下的角度 tool_tilt_from_down_deg(0 = 夹爪正朝下),以及夹爪状态。不会让机器人运动。"),
        _tool("move_arm_end",
              "让一条手臂的末端相对当前位置平移和/或转动,姿态不转时保持不变。"
              "先规划直线,直线会撞就自动规划绕障路径;目标本身被点云当成障碍(要碰到物体)时,"
              "会先避障到目标前几厘米,最后一小段不查外界碰撞慢速直线过去。会让机器人运动。"
              "只给 tool_axis 不给 offset_cm 时,程序会自己判断:原地转不到就往身体方向后撤着转,"
              "不用你决定撤多少;结果里 retreat_m 是实际后撤的距离、tilt_fraction 是转到了几成(1 = 完全到位)。"
              "base_link 坐标系:x 向前,y 向左,z 向上。",
              {"arm": {"type": "string", "enum": ["left", "right"]},
               "offset_cm": _xyz("相对当前末端的平移 [dx, dy, dz],单位厘米。例如往上抬 10 厘米是 [0, 0, 10]"),
               "rotate_deg": _xyz("绕 base_link 的 x、y、z 轴依次转多少度。只在用户给出具体转角时用;"
                                  "“夹爪朝下/朝前”用 tool_axis,不要自己换算成转角"),
               "tool_axis": {"type": "string", "enum": ["down", "up", "forward", "backward", "left", "right"], "description": "让夹爪(工具轴)朝 base_link 的哪个方向,例如夹爪竖直朝下填 down;手腕不会自转。不需要时不填"},
               "wrist_roll_deg": {"type": "number", "description": "手腕绕自身工具轴转多少度(右手定则),不填 = 不自转;只有用户明确要求转手腕时才填"}},
              required=["arm"]),
        _tool("move_joints",
              f"让一组关节转动:{GROUP_TEXT}。沿途做整机碰撞检查,会撞就自动绕行。"
              "deltas_deg 和 angles_deg 只填一个,长度等于这一组的关节数,不动的关节填 0(deltas)或当前角度(angles)。"
              "会让机器人运动。",
              {"group": {"type": "string", "enum": list(GROUPS)},
               "deltas_deg": {"type": "array", "items": {"type": "number"},
                              "description": "每个关节在当前角度上再转多少度"},
               "angles_deg": {"type": "array", "items": {"type": "number"},
                              "description": "每个关节转到多少度(绝对角度)"}},
              required=["group"]),
    ]
    tools.append(_tool(
        "gripper",
        "打开或闭合一只手的夹爪。会让机器人动作。失败时(比如这只手没装夹爪)如实告诉用户。",
        {"arm": {"type": "string", "enum": ["left", "right"], "description": "哪只手,用户说右手就是 right"},
         "action": {"type": "string", "enum": ["open", "close"], "description": "open 打开,close 闭合"}},
        required=["arm", "action"]))
    tools.append(_tool(
        "wait",
        "等待几秒再做下一步,例如“打开夹爪,过 5 秒再闭合”中间的 5 秒。不会让机器人运动。",
        {"seconds": {"type": "number", "description": "等多少秒"}},
        required=["seconds"]))
    if pose_names:
        listing = ""
        for name in pose_names:
            note = (pose_notes or {}).get(name)
            if note:
                listing += f"{name} = {note};"
        tools.append(_tool(
            "go_to_pose",
            "让机器人回到一个事先录好的全身位姿。沿途做整机碰撞检查,会撞就经其他录好的位姿中转或自动绕行。"
            "会让机器人运动。" + (f"各位姿的含义:{listing}" if listing else ""),
            {"name": {"type": "string", "enum": list(pose_names)},
             "groups": {"type": "array", "items": {"type": "string", "enum": list(GROUPS)},
                        "description": "只动其中几组,不填就是这个位姿包含的所有组"}},
            required=["name"]))
    return tools
