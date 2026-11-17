# experience/：给大模型看的经验

SYSTEM 里只放一份目录（每条经验的名字加一句话说明什么时候用）。大模型觉得相关时，调用 `read_experience(name)` 读全文；不相关就不读，不占上下文。代码在 [harness/experience.py](../harness/experience.py)。

经验是**推荐做法**，大模型可以变通。违反了就危险的规则不放这里，放在程序（gateway）里强制执行。

## 一条经验的格式

一个 markdown 文件，文件名就是名字：

```markdown
---
name: pick_up_object              # 和文件名一致,只用小写字母、数字、下划线
when: 从桌上拿起 / 抓起一个东西     # 一句话:什么时候用(会出现在 SYSTEM 的目录里)
status: 真机成功 1 次              # 真机成功 N 次 / 未在真机验证 / 已知有问题
source: traces/20261114_032417.jsonl   # 从哪次 trace 总结出来的,可以写多个
---
## 原则       为什么这样做(写原则,模型才能迁移到没见过的任务)
## 做法       步骤,写清楚用哪个工具、哪些参数
## 常见坑     踩过的坑,附出处
## 怎么确认成功
```

## 经验从哪里来

| 来源 | 去处 |
|---|---|
| 人复盘真机 trace 后写 | 直接放进本目录 |
| 大模型在任务里调用 `propose_experience` | `experience/pending/`，**不生效**，人看过后用 `scripts/experience.py approve <名字>` 移进本目录 |

大模型的自我总结可能是错的（2026-11-14 它确信“已经放到您手上了”，其实没有），所以必须人审。

```bash
python3 scripts/experience.py list              # 正式的和待审的
python3 scripts/experience.py show <名字>
python3 scripts/experience.py approve <名字>    # pending/ → 正式
python3 scripts/experience.py reject <名字>     # 删掉待审的
```
