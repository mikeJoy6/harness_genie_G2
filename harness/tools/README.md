# harness/tools/ —— 给模型看的“工具说明书”

## 这里干什么

模型看不到你的代码,只能看到这里写的工具名、说明和参数,并靠这些决定调用什么。这里不执行任何动作。

## 这里的文件

| 文件 | 工具 | 会让机器人动吗 |
|---|---|---|
| `objects.py` | `find_objects`:SAM 3 按英文名找物体,返回编号 | 不会 |
| | `approach_object`:末端到物体正上方(默认右臂) | **会** |
| | `touch_object`:夹爪尖碰到物体(唯一能碰物体的:最后几厘米只放过物体附近的点云) | **会** |
| `motion.py` | `get_robot_state`:所有关节角 + 两臂末端位姿 | 不会 |
| | `move_arm_end`:末端相对平移 / 转动,直线不行自动绕障 | **会** |
| | `move_joints`:腰 / 头 / 左臂 / 右臂某一组关节转动 | **会** |
| | `go_to_pose`:回到 `configs/poses.yaml` 里录好的位姿 | **会** |

`objects.py` 里的 `make_tools(位姿名, 位姿说明)` 把两个文件的工具合在一起。没有录位姿时不提供 `go_to_pose`。

## 和哪里联动

```text
tools/objects.py + motion.py ─→ loop.py 每轮发给模型
                       │ 模型回复要调用的工具名
                       ▼
                  gateway.py 用同样的名字找到处理函数
```

**工具名必须和 `gateway.py` 的 `handlers` 一字不差**,新增工具时两边都要改。

## 示例:工具的格式

模型给不了绝对坐标:`approach_object` 的目标点由 `find_objects` 的编号查出,`move_arm_end` 只收相对偏移:

```python
_tool("move_arm_end",
      "让一条手臂的末端相对当前位置平移和/或转动……会让机器人运动。",
      {"arm": {"type": "string", "enum": ["left", "right"]},
       "offset_cm": _xyz("相对当前末端的平移 [dx, dy, dz],单位厘米……"),
       "rotate_deg": _xyz("绕 base_link 的 x、y、z 轴依次转多少度,不转就不填")},
      required=["arm"])
```
