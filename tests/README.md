# tests/ —— 不花钱、不动机器人的自动检查

## 这里干什么

用“照剧本回复的假模型”和“假机器人”检查 harness 的逻辑和安全规则。不需要 API key,也不需要 ROS。

## 这里的文件

| 文件 | 作用 |
|---|---|
| `test_objects.py` | 完整流程、安全拒绝、颜色识别和取色的测试,不需要 ROS |
| `test_motion.py` | 运动学、IK、积分笛卡尔、`via_then_rrt` 绕障;需要 ROS 环境和 G2 的 URDF,缺了就跳过 |

## 和哪里联动

```text
tests/test_objects.py
   ├─ ScriptedLLM              代替 harness/llm.py
   ├─ harness/backends/fake.py 代替真机
   ├─ harness/backends/color_object.py
   ├─ harness/gateway.py
   ├─ harness/loop.py
   └─ configs/harness.yaml     使用真实的限制值
```

## 示例

```python
def test_cannot_move_without_locating():
    backend = FakeBackend()
    assert not Gateway(backend, MOTION, COLORS).execute("approach_object", {})["ok"]
    assert backend.executed == []      # 重点:机器人确实没动
```

运行:

```bash
cd ~/robot_harness
python3 -m pytest tests -q
```

每次改 `gateway.py`、`loop.py` 或 `configs/harness.yaml` 后都跑一遍。
