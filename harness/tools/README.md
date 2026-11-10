# harness/tools/ —— 给模型看的“工具说明书”

## 这里干什么

模型看不到你的代码,只能看到这里写的工具名、说明和参数,并靠这些决定调用什么。这里不执行任何动作。

## 这里的文件

`objects.py` 定义了三个工具,颜色选项来自 `configs/colors.yaml`:

| 工具 | 会让机器人动吗 |
|---|---|
| `locate_object` | 不会 |
| `get_left_end_pose` | 不会 |
| `approach_object` | **会**,且需要你按 y |

## 和哪里联动

```text
tools/objects.py ─→ loop.py 每轮发给模型
                       │ 模型回复要调用的工具名
                       ▼
                  gateway.py 用同样的名字找到处理函数
```

**工具名必须和 `gateway.py` 的 `handlers` 一字不差**,新增工具时两边都要改。

## 示例:工具的格式

`approach_object` 没有 x、y、z 参数。模型无法指定坐标,只能选择要不要靠近、停多高:

```python
_tool("approach_object",
      "让左臂末端沿直线移动到最近一次定位到的物体正上方。会让机器人运动。",
      {"standoff_cm": {"type": "number", "description": "末端停在物体上方多少厘米"}})
```
