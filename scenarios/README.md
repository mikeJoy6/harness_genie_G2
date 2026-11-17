# scenarios/：检查真大模型的表现

单元测试（`tests/`）用写死的模型回复，只检查程序逻辑。这里用**真的 DeepSeek** + 会动的假机器人，检查模型在一条指令下实际会调用哪些工具。改 SYSTEM、改工具说明、加经验之后，跑一遍就知道是变好还是变坏。机器人不会动，但会调 DeepSeek（花一点钱）。

| 文件 | 作用 |
|---|---|
| `*.yaml` | 一个场景：指令、假场景里有哪些东西、检查哪些调用 |
| `fake_world.py` | 会动的假机器人：闭合夹爪时夹爪尖 3 cm 内的东西被拿起来、跟着夹爪走；张开时落下（下面有东西就落在它上面） |
| `baseline_system.txt` | 2026-11-16 改写前的 SYSTEM，当对照组 |

```bash
cd ~/robot_harness
python3 scripts/run_scenarios.py                         # 所有场景,新 SYSTEM + 经验,每个 3 遍
python3 scripts/run_scenarios.py handover --runs 5       # 只跑名字里带 handover 的
python3 scripts/run_scenarios.py --variant baseline      # 旧 SYSTEM、没有经验
python3 scripts/run_scenarios.py --variant no_experience # 新 SYSTEM、没有经验
```

对话存在 `traces/scenarios/<时间>_<变体>/`。场景文件格式和检查类型见 [scripts/run_scenarios.py](../scripts/run_scenarios.py) 文件头。

## 联动

```text
scenarios/*.yaml → scripts/run_scenarios.py → harness/loop.py(run_agent)+ scenarios/fake_world.py + experience/
```
