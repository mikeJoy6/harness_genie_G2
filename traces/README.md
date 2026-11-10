# traces/ —— 每次运行的完整记录

## 这里干什么

程序自动记录模型的每句话、每次工具调用和每个结果。出了问题先看这里。

## 这里的文件

| 文件 | 作用 |
|---|---|
| `<年月日_时分秒>.jsonl` | 一条指令的记录,一行一个事件 |
| `last_object.jpg` | 最近一次定位时的画面,识别到的物体有绿框 |

## 和哪里联动

```text
scripts/run_task.py 打开文件 → harness/loop.py 逐行写入
harness/backends/ros.py → last_object.jpg
```

## 示例

查看最近一次运行:

```bash
cd ~/robot_harness
python3 - <<'EOF'
import json
from pathlib import Path

latest = sorted(Path("traces").glob("*.jsonl"))[-1]
for line in latest.read_text(encoding="utf-8").splitlines():
    event = json.loads(line)
    print(f"[{event['kind']}]", json.dumps(event["data"], ensure_ascii=False))
EOF
```

| 现象 | 看哪里 | 通常要改 |
|---|---|---|
| 模型选错工具 | `assistant` | `harness/tools/objects.py` 的描述 |
| 找不到物体 | `tool` 和 `last_object.jpg` | 用 `scripts/hsv_pick.py` 重新取色 |
| 规划失败 | `tool` | 物体位置或手臂起始姿态 |
| 工具失败,模型却说成功 | 最后的 `assistant` | `harness/loop.py` 的 `SYSTEM` |
