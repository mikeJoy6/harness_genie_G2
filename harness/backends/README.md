# harness/backends/ —— 真正执行动作的地方

## 这里干什么

Gateway 决定“能不能做”,这里负责“具体怎么做”。假机器人和真机器人提供同样的方法,上层不用改。

## 这里的文件

| 文件 | 作用 | 依赖 ROS |
|---|---|---|
| `color_object.py` | 从一帧 RGBD 按 HSV 颜色找物体 | 否 |
| `fake.py` | 假机器人,用于测试 | 否 |
| `ros.py` | 真机器人 | 是 |

## 和哪里联动

```text
harness/gateway.py
   ├─ backends/fake.py
   └─ backends/ros.py
        ├─ color_object.py                     按颜色找物体
        ├─ agi@g2_robot 的 RosSource、Rgbd   相机取图、深度转 base_link
        ├─ agi@g2_robot 的 ArmMotion         规划、自碰撞检查、发给 JTC
        └─ 发布 TF:base_link → target_object    可在 RViz 里查看
```

`ros.py` 直接 import 已安装的 `agi@g2_robot` 包,所以运行前要 `source ~/agi@g2_robot/env_g2.sh`。

## 两个 backend 都要有的方法

```python
locate_object(color, ranges)  # 返回 {"ok": True, "xyz_m": [x, y, z], ...}
left_end_xyz()          # 返回 [x, y, z]
plan_left_to(target)    # 返回 {"ok": True, "summary": "..."}
execute(plan)           # 返回 {"ok": True, "msg": "..."}
close()
```

## 示例:物体识别不到时

看 `traces/last_object.jpg`,识别到的物体上有绿框。没有绿框时,用点选工具重新取色:

```bash
python3 scripts/hsv_pick.py --name 青椒
```
