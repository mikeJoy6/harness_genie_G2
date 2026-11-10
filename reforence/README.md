# reforence/ —— 只读参考资料

## 这里干什么

放手写时要对照的资料。**程序不会读取这里的任何文件**,也不要从这里 import 代码。

这里的都是快照,以 `agi@g2_robot` 的原文件为准。

## 建议放的文件

| 文件 | 用途 |
|---|---|
| `TASKS.md` | 吸纸板的完整任务、已完成和未完成的部分 |
| `run.txt` | 启动真机和各种工具的命令 |
| `protocol.py` | 请求和状态的格式,写 backend 时对照 |
| `base.py` | 任务基类,以后新增机器人技能时对照 |
| `ai_assistant_README.md` | 真机安全规则和机器人开发知识的入口 |

## 和哪里联动

```text
agi@g2_robot/TASKS.md、run.txt            ┐
agi@g2_robot/.../common/protocol.py       ├─ 手动 cp ─→ reforence/ ─ 人阅读
agi@g2_robot/.../tasks/base.py            │
ai_assistant/README.md                    ┘

真正的运行依赖不经过这里:
harness/backends/ros.py ─ import ─→ agi@g2_robot 已安装的 task_board_pose
```

## 示例

拷贝资料:

```bash
cd ~/robot_harness/reforence
G2=~/agi@g2_robot
cp $G2/TASKS.md $G2/run.txt .
cp $G2/src/task_board_pose/task_board_pose/common/protocol.py .
cp $G2/src/task_board_pose/task_board_pose/tasks/base.py .
cp ~/ai_assistant/README.md ai_assistant_README.md
```

`protocol.py` 里最需要看懂的两条消息:

```json
{"id": "a1b2c3", "task": "goto_pose", "args": ["look", "groups=head"]}
{"id": "a1b2c3", "task": "goto_pose", "state": "done", "msg": "到 'look'", "result": {"name": "look"}}
```

第一条发到 `/task_board_pose/request`,第二条从 `/task_board_pose/status` 收到。

手动给 processor 发一条任务,可以感受这个协议(**这会让真机运动**,确认现场安全后再执行):

```bash
source ~/agi@g2_robot/env_g2.sh
ros2 topic pub --once /task_board_pose/request std_msgs/String "{data: 'goto_pose look groups=head'}"
ros2 topic echo /task_board_pose/status
```

`ai_assistant/README.md` 里这条规则对 harness 同样有效:**AI 不得自行启动真机 launch**。
所以 `harness/tools/` 里永远不提供启动 launch 的工具。
