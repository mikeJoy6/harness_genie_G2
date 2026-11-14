# harness/ —— Harness 的核心

## 这里干什么

模型只会输出文字。这里的代码把模型的话变成受控的动作,再把结果告诉模型。

你说“右手去手机上方”后,实际发生的事:

```text
模型:调用 find_objects(phone)
程序:SAM 3 分割 + 深度算出每个手机的坐标,存进编号表,把 phone_1、phone_2 告诉模型
模型:调用 approach_object(object_id=phone_1)
程序:按编号查坐标 → 算目标点 → 安全检查 → 规划 → 执行 → 告诉模型结果
模型:没有工具要调用了,用文字告诉你结果      ← 循环结束
```

模型只决定“下一步调用什么”,坐标全部由程序计算。

## 这里的文件

| 文件 | 作用 |
|---|---|
| `loop.py` | 循环:问模型 → 执行工具 → 把结果告诉模型 |
| `llm.py` | 通过 HTTP 调用 DeepSeek |
| `gateway.py` | 关卡:计算目标点,所有安全检查都在这里 |
| `motion/` | 运控与绕障规划,见其 README |
| `perception/` | 头部 RGBD,见其 README |

## 和哪里联动

```text
scripts/run_task.py  组装下面所有东西
loop.py ── chat() ──→ llm.py ──→ DeepSeek
loop.py ── tools/objects.py 告诉模型有哪些工具
loop.py ── execute() ──→ gateway.py ──→ backends/fake.py 或 ros.py
                                          ros.py ──→ motion/(规划、执行)+ perception/(相机)
loop.py ── 每一步写入 traces/
configs/harness.yaml ── 提供 gateway 的限制值和 llm 的模型名
```

## 示例:加一个新的安全检查

所有运动前的检查都在 `gateway.py` 的 `_approach` 里,按这个形式添加:

```python
if target[2] > 1.0:
    return {"ok": False, "error": "目标太高"}    # 错误会交给模型,由模型告诉你
```
