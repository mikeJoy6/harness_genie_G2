# scripts/ —— 从命令行启动

## 这里干什么

读配置、选择假机器人或真机、启动循环。这里不放业务逻辑。

## 这里的文件

| 文件 | 作用 |
|---|---|
| `run_task.py` | 启动 harness |
| `hsv_pick.py` | 在头部画面上点选颜色,存入 `configs/colors.yaml` |

## 和哪里联动

```text
scripts/run_task.py
   ├─ 读 configs/harness.yaml
   ├─ 创建 harness/llm.py
   ├─ 创建 backends/fake.py,加 --real 时换成 ros.py
   ├─ 每条指令创建一个新的 harness/gateway.py
   ├─ 调用 harness/loop.py
   └─ 写入 traces/<时间>.jsonl
```

## 示例

```bash
cd ~/robot_harness

# 假机器人
python3 scripts/run_task.py "让左臂末端靠近红色积木"

# 真机:只规划,绝不运动
python3 scripts/run_task.py --real --plan-only "让左臂末端靠近红色积木"

# 真机:交互模式,一行一条指令,输入 q 退出
python3 scripts/run_task.py --real
```

新增一种颜色(左键点物体,`s` 保存,`q` 退出):

```bash
source ~/agi@g2_robot/env_g2.sh
python3 scripts/hsv_pick.py --name 青椒
python3 scripts/run_task.py --real --plan-only "左手抵达青椒上空"
```

真机启动步骤见项目根目录 `run.txt`。
