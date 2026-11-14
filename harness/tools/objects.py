"""给模型看的工具说明。工具名必须和 harness/gateway.py 里的一致。
物体相关的工具在这里,运动工具在 motion.py,make_tools 把两边合起来。"""
from .motion import make_motion_tools


def _tool(name, description, properties=None, required=()):
    return {"type": "function", "function": {
        "name": name, "description": description,
        "parameters": {"type": "object", "properties": properties or {}, "required": list(required)}}}


def make_tools(pose_names=(), pose_notes=None):
    return _object_tools() + make_motion_tools(pose_names, pose_notes)


def _object_tools():
    return [
        _tool("approach_object",
              "让手臂末端(夹爪尖)移动到 find_objects 找到的物体正上方,不碰到它;先规划直线,会撞就自动绕障。"
              "会让机器人运动。",
              {"object_id": {"type": "string", "description": "find_objects 返回的编号,例如 phone_1"},
               "tool_axis": {"type": "string", "enum": ["down", "up", "forward", "backward", "left", "right"], "description": "让夹爪(工具轴)朝 base_link 的哪个方向,例如夹爪竖直朝下填 down;手腕不会自转。不需要时不填"},
               "wrist_roll_deg": {"type": "number", "description": "手腕绕自身工具轴转多少度(右手定则),不填 = 不自转;只有用户明确要求转手腕时才填"},
               "standoff_cm": {"type": "number",
                               "description": "夹爪尖停在物体上方多少厘米(3~30,不填 = 12);只有用户明确要求时才填"},
               "arm": {"type": "string", "enum": ["left", "right"],
                       "description": "用哪条手臂,不填就是右臂(只有右手装了夹爪);用户说左手才填 left"}},
              required=["object_id"]),
        _tool("find_objects",
              "用头部相机 + SAM 3 按物体名找画面里所有这种物体(任何物体都行,不限颜色)。"
              "返回每个物体的编号 object_id、base_link 坐标、离相机的距离、离画面中心的像素偏差,"
              "列表已按离相机由近到远排好,第一个最近。不会让机器人运动。",
              {"query": {"type": "string",
                         "description": "英文名词短语,例如 phone、cup、red block、pen、hand、person。"
                                        "用户说中文时翻译成英文,只写物体名,不要写“最近的”之类的修饰"}},
              required=["query"]),
        _tool("point_at_object",
              "让一只手的夹爪指向 find_objects 找到的物体(比如人的手):末端位置基本不动,只把夹爪方向转过去,"
              "手腕不自转;转不到时程序会自己往身体方向后撤一点再转。会让机器人运动。",
              {"object_id": {"type": "string", "description": "find_objects 返回的编号,例如 hand_1"},
               "arm": {"type": "string", "enum": ["left", "right"], "description": "哪只手,用户说右夹爪就是 right"}},
              required=["object_id", "arm"]),
        _tool("touch_object",
              "让一只手的夹爪尖碰到 find_objects 找到的物体,或停在它跟前几厘米:用于击掌、碰拳、摸一下、"
              "“指着它并离它 1 厘米”、抓取时下探到物体。唯一会接触物体的工具。"
              "夹爪自动朝向物体,夹爪尖停在物体表面前 gap_cm 处,不用你算偏移;"
              "程序先避障到物体前几厘米,最后一小段慢速直线贴过去(这段不查点云)。会让机器人运动。",
              {"object_id": {"type": "string", "description": "find_objects 返回的编号,例如 hand_1"},
               "arm": {"type": "string", "enum": ["left", "right"], "description": "用哪只手,先看 reachable_by"},
               "gap_cm": {"type": "number", "description": "夹爪尖离物体表面留多少厘米,可填 0~10。"
                                                           "用户说了距离就照填(“离它 1 厘米”填 1、“贴上去”填 0);没说就不填(= 2)"}},
              required=["object_id", "arm"]),
        _tool("look_at_object",
              "转头(只动头部 3 个关节,不动腰和手臂),让 find_objects 找到的某个物体位于画面中央;"
              "给 object_ids 时对准这几个物体的中间(比如拍一群人前先对准人群)。"
              "只看一个物体时转完会再识别一次,返回离画面中心的像素偏差。会让机器人运动。",
              {"object_id": {"type": "string", "description": "find_objects 返回的编号,例如 phone_1"},
               "object_ids": {"type": "array", "items": {"type": "string"},
                              "description": "多个编号,对准它们的中间;和 object_id 二选一"}}),
        _tool("take_photo",
              "用机器人头部相机拍一张照片存到电脑上(返回文件路径)。机器人不会运动。"
              "head_wide 是 1920x1536 广角,适合拍人、拍全景(默认);head 是 640x400。"
              "拍人或某个物体前,可以先 find_objects 再 look_at_object 把头对准。",
              {"camera": {"type": "string", "enum": ["head_wide", "head"], "description": "不填 = head_wide"}}),
    ]
