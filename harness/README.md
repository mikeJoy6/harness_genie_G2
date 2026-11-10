# harness/ —— Harness 的核心

## 这里干什么

模型只会输出文字。这里的代码把模型的话变成受控的动作,再把结果告诉模型。

你说“让左臂末端靠近红色积木”后,实际发生的事:

```text
模型:调用 locate_object
程序:用头部相机算出积木坐标,发布 TF target_object,把坐标告诉模型
模型:调用 get_left_end_pose
程序:读出 arm_l_end_link 坐标,告诉模型
模型:调用 approach_object
程序:算目标点 → 安全检查 → 规划 → 等你按 y → 执行 → 告诉模型结果
模型:没有工具要调用了,用文字告诉你结果      ← 循环结束
```

模型只决定“下一步调用什么”,坐标全部由程序计算。

## 这里的文件

| 文件 | 作用 |
|---|---|
| `loop.py` | 循环:问模型 → 执行工具 → 把结果告诉模型 |
| `llm.py` | 通过 HTTP 调用 DeepSeek |
| `gateway.py` | 关卡:计算目标点,所有安全检查都在这里 |

## 和哪里联动

```text
scripts/run_task.py  组装下面所有东西
loop.py ── chat() ──→ llm.py ──→ DeepSeek
loop.py ── tools/objects.py 告诉模型有哪些工具
loop.py ── execute() ──→ gateway.py ──→ backends/fake.py 或 ros.py
loop.py ── 每一步写入 traces/
configs/harness.yaml ── 提供 gateway 的限制值和 llm 的模型名
```

## 示例:加一个新的安全检查

所有运动前的检查都在 `gateway.py` 的 `_approach` 里,按这个形式添加:

```python
if target[2] > 1.0:
    return {"ok": False, "error": "目标太高"}    # 错误会交给模型,由模型告诉你
```
