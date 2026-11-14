# harness/backends/ —— 真正执行动作的地方

## 这里干什么

Gateway 决定“能不能做”,这里负责“具体怎么做”。假机器人和真机器人提供同样的方法,上层不用改。

## 这里的文件

| 文件 | 作用 | 依赖 ROS |
|---|---|---|
| `gripper_gdk.py` | 夹爪开合(GDK) | 是 |
| `fake.py` | 假机器人,用于测试 | 否 |
| `ros.py` | 真机器人 | 是 |

## 和哪里联动

```text
harness/gateway.py
   ├─ backends/fake.py
   └─ backends/ros.py
        ├─ harness/perception/             相机取图、SAM 3 分割、深度转 base_link
        ├─ harness/motion/ArmMotion        直线规划 → 失败时 via_then_rrt 绕障 → 整机碰撞(含点云)→ JTC
        ├─ harness/motion/WholeBody        腰 / 头 / 双臂关节运动
        └─ gripper_gdk.py                  夹爪
```

`ros.py` 只 import 本项目的代码。运行前仍要 `source ~/agi@g2_robot/env_g2.sh`,那是为了 GDK 的 DDS 配置,不是为了 import 代码。

## 两个 backend 都要有的方法

```python
find_objects(query)                   # SAM 3:{"ok": True, "objects": [{"id", "xyz_m", ...}]}
end_pose(arm)                         # {"xyz_m": [x, y, z], "rpy_deg": [r, p, y], "link"},arm = "left" / "right"
joint_state()                         # {关节名: 弧度}
joint_limits()                        # {关节名: (下限, 上限)}
plan_arm_to(arm, target, ...)         # 只规划,全程查碰撞:{"ok": True, "kind": "arm" / "body_then", "summary", "lowest_z_m"}
plan_touch(arm, object_xyz, gap_m, direction)   # 只规划 touch_object(唯一不查点云的):{"kind": "touch", ...}
plan_joints(target, speed)            # 只规划:{"ok": True, "kind": "joints", "summary": ...}
execute(plan)                         # 按 plan["kind"] 执行:{"ok": True, "msg": "..."}
close()
```

## 示例:物体识别不到时

看 `traces/sam3/<时间>_<短语>/overlay.jpg`,上面画了每个掩码;没有掩码就换个英文短语,或用 `scripts/sam3_live.py` 实时试。
