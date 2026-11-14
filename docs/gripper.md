# 夹爪(末端执行器)

对 harness 说“打开右手夹爪”“你先打开夹爪过 5 秒再闭合”,模型调用 `gripper(arm, action)` 和 `wait(seconds)`。两只手按都装了夹爪来设计;没装或报错的手,工具返回失败和原因,模型如实转告。

## 夹爪在哪里控制

`ros2 topic list` 里**没有**夹爪话题:G2 的 `/gdk/joint_state` 不含夹爪,`hardware.launch` 的 ros2_control 插件也不管夹爪(`arm_control_ros.cpp` 注释第 5 条)。夹爪只能走 GDK:

| 用途 | 接口 | 说明 |
|---|---|---|
| 型号、错误码 | `agibot_gdk.Robot().get_whole_body_status()` | `{left,right}_end_model`、`{left,right}_end_error` |
| 夹爪位置 | `Robot().get_end_state()`(默认不读,`read_position: false`) | 2026-11-12 实测一直 `No end state message found`,读不到。每调一次 GDK 都往终端打一行 `E... hal_impl.cpp:305 No end state message found`(不是 harness 报错);`GdkGripper` 读不到一次后就不再读 |
| 下发(默认) | 话题 `/gdk/joint_control`(`gdk_msgs/JointPositionServo`),只带夹爪关节 | 手臂插件也在发这个伺服话题;GDK 文档:与伺服接口同时用时,末端要走伺服接口 |
| 下发(备选) | `Robot().move_ee_pos(JointStates(group="right_tool"))` | 非阻塞;文档说不能和伺服接口同时用 |

关节名(`~/.cache/agibot/app/gdk/config/joint_mapping.json`):右 `idx71_gripper_r_inner_joint1`,左 `idx31_gripper_l_inner_joint1`。
取值(GDK 文档):omnipicker −0.785 开 / 0 合;ctek90d −0.91 / 0;dahuan 0 / 0.025(米)。

真机现状(`gripper_cli.py status`):右手 `omnipicker`、错误码 0;左手型号为空、错误码 2147549184。

## 代码

| 文件 | 作用 |
|---|---|
| [harness/backends/gripper_gdk.py](../harness/backends/gripper_gdk.py) | `GdkGripper`:第一次用时才 `gdk_init`;`state(arm)`、`command(arm, action)`(查型号/错误码 → 下发 → 等 `settle_s` → 再读错误码/位置) |
| [harness/gateway.py](../harness/gateway.py) | `_gripper`:算一次运动,只规划模式不动,和其他运动一样默认自动执行(`confirm_motion: true` 或 `--confirm` 时按 y);`_wait`:0~`max_wait_s` 秒;`get_robot_state` 带上两只手的夹爪状态 |
| [harness/tools/motion.py](../harness/tools/motion.py) | 工具 `gripper(arm, action)`、`wait(seconds)` |
| [scripts/gripper_cli.py](../scripts/gripper_cli.py) | 不经过大模型单独测:`status` / `open` / `close`,`--interface` 切换下发方式 |
| [configs/harness.yaml](../configs/harness.yaml) | `motion.gripper`(interface、confirm、servo_hold_s、servo_rate_hz、settle_s、position_tol)、`motion.max_wait_s` |

## 结果怎么判断

- 型号为空 / 不支持 / 有错误码:**不下发**,返回失败和原因(左手现在就是这样)。
- 下发后又出错误码:失败。
- 读得到位置:离目标 ≤ `position_tol` 算到位,否则失败(可能夹住了东西)。
- 读不到位置(现状):`ok: true, verified: false`,说明“命令已发送、没报错,但确认不了到位”。

## 没验证的

- **两种下发方式都还没在真机上动过**。伺服方式只发夹爪一个关节:GDK 会不会因为这包没带手臂关节而影响手臂,要先在手臂不动、旁边有人时用 `gripper_cli.py` 试。
- `move_ee_pos` 和正在跑的手臂伺服同时用会怎样,文档只说“不能”,没说后果;只在 servo 不行时再试。
