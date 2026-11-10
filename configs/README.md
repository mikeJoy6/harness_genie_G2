# configs/ —— 不写死在代码里的设置

## 这里干什么

存放经常要改的值:模型、运动速度、安全范围、视觉阈值。改完不用编译,下次运行生效。

**API key 不要写在这里**,这里只写环境变量的名字。

## 这里的文件

| 文件 | 作用 |
|---|---|
| `harness.yaml` | 模型、运动、视觉的配置 |
| `colors.yaml` | 可识别的颜色,由 `scripts/hsv_pick.py` 添加 |

## 和哪里联动

```text
configs/harness.yaml ← scripts/run_task.py 读取
   llm.*     → harness/llm.py
   agent.*   → harness/loop.py
   motion.*  → harness/gateway.py,speed_m_s 也给 backends/ros.py
   vision.*  → harness/backends/ros.py 和 color_object.py
```

## 示例

让手臂更慢,停得更高:

```yaml
motion:
  speed_m_s: 0.03
  standoff_default_m: 0.30
```

设置 key(只在当前终端有效):

```bash
export DEEPSEEK_API_KEY=你的key
```
