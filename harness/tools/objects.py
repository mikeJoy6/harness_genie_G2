"""给模型看的工具说明。工具名必须和 harness/gateway.py 里的一致。"""


def _tool(name, description, properties=None, required=()):
    return {"type": "function", "function": {
        "name": name, "description": description,
        "parameters": {"type": "object", "properties": properties or {}, "required": list(required)}}}


def make_tools(color_names):
    return [
        _tool("locate_object",
              "用头部 RGBD 相机按颜色定位物体,返回它在 base_link 下的坐标,并发布 TF target_object。"
              "同色物体有多个时,返回离左臂末端最近的那个。不会让机器人运动。",
              {"color": {"type": "string", "enum": list(color_names),
                         "description": "物体的颜色名称,例如青椒选 green"}},
              required=["color"]),
        _tool("get_left_end_pose",
              "读取左臂末端 arm_l_end_link 当前在 base_link 下的坐标。不会让机器人运动。"),
        _tool("approach_object",
              "让左臂末端沿直线移动到最近一次定位到的物体正上方。会让机器人运动。",
              {"standoff_cm": {"type": "number",
                               "description": "末端停在物体上方多少厘米,可不填;只有用户明确要求时才填"}}),
    ]
