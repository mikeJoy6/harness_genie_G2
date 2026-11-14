# 物体编号表(handle):识别结果怎么交给运动工具

问题:最终想要 `plan(物品A)`。模型知道自己能 plan,但“物品A”怎么作为参数传进去?不能把所有物体都写进工具定义,也不能让模型抄坐标。

答案:**识别工具返回编号,运动工具只收编号**。坐标始终留在 gateway 内部。

> 状态:**部分实现**:`find_objects` / `look_at_object` 已经用编号表(`Gateway.objects`),见 [look_at_object.md](look_at_object.md);`approach_object` 填 `object_id` 时也查编号表;不填时仍用 `locate_object`(按颜色)的单槽位 `self.target`。识别来源见 [perception_plan.md](perception_plan.md)。

## 现状:只有一个槽位

[`Gateway`](../harness/gateway.py) 用 `self.target`([gateway.py:46](../harness/gateway.py#L46))存“最近一次定位到的物体”:

- [`_locate`](../harness/gateway.py#L69) 成功时写入([:76](../harness/gateway.py#L76)),失败时清空([:73](../harness/gateway.py#L73));
- [`_approach`](../harness/gateway.py#L193) 读它算目标点([:216](../harness/gateway.py#L216))。

一次一个物体够用;“先看红积木,再看笔,再去红积木那里”做不到,第二次识别会覆盖第一次。

## 设计

```text
用户:"把左手移到笔上面"
 ① locate_object(query="pen")                 参数是自由文本,不是枚举
      识别器找到 → gateway 存:
      self.objects["obj_1"] = {"xyz": [0.79, 0.10, 0.77], "time": t, "label": "pen"}
      返回模型:{"ok": true, "id": "obj_1", "label": "pen", ...}
 ② approach_object(object_id="obj_1")          只传编号
      gateway 查表 → 算目标点 → plan_arm_to → plan_detour
```

三个要点:

1. **识别工具参数是自由文本**。物体种类由识别器决定,工具定义不用列。现在的颜色识别器只能映射到颜色,所以暂时是颜色枚举;换 SAM 3 后定义不用改。
2. **运动工具收编号,gateway 查表并校验**:

   ```python
   obj = self.objects.get(args.get("object_id"))
   if obj is None:
       return {"ok": False, "error": f"没有这个物体,已识别的:{list(self.objects)}"}
   if time.monotonic() - obj["time"] > 30:
       return {"ok": False, "error": "这个物体的位置已过期,重新识别"}
   ```

   模型编一个不存在的编号,直接拒绝并告诉它有哪些可选。
3. **编号在识别那一刻绑定坐标**。点云聚类(DBSCAN)每帧编号都会变,不能拿同一个编号去查新的一帧。

`self.objects` 就是一张小“世界模型”,以后可以加 `list_objects()`、一次识别多个、抓起后更新位置。

## 要改的地方

| 文件 | 改动 |
|---|---|
| [harness/gateway.py](../harness/gateway.py) | `self.target` → `self.objects` 字典;`_locate` 返回 `id`;`_approach` 收 `object_id` |
| [harness/tools/objects.py](../harness/tools/objects.py) | `approach_object` 加 `object_id` 参数;`locate_object` 改自由文本(换识别器时) |
| [tests/test_objects.py](../tests/test_objects.py) | 多物体、过期、编号不存在 |

识别器和规划器不用动。

## 相关论文

| 论文 | 和本问题的关系 |
|---|---|
| **Code as Policies**(Liang 等,ICRA 2023)[arXiv:2209.07753](https://arxiv.org/abs/2209.07753) · [项目页](https://code-as-policies.github.io) | 物体按**名字字符串**传:`get_obj_names()`、`get_obj_pos(name)`、`put_first_on_second('gray block', 'gray bowl')`,底层开放词汇检测器(ViLD、MDETR)。区别:它让模型写代码,我们让模型调工具。重点读第 III 节 |
| **SayPlan**(Rana 等,CoRL 2023)[arXiv:2307.06135](https://arxiv.org/abs/2307.06135v2) · [项目页](https://sayplan.github.io) | 物体很多时(3 层楼、140 个物体)场景存成 3D 场景图,模型用 `expand`/`contract` 只展开相关部分,路径由经典规划器补全。是编号表的大规模版本(只读了摘要和项目页) |

harness 整体结构的来源:

| 组成 | 已有做法 |
|---|---|
| 问模型 → 调工具 → 看结果 → 再问 | ReAct [arXiv:2210.03629](https://arxiv.org/abs/2210.03629) |
| 模型只选技能,技能由程序实现 | SayCan [arXiv:2204.01691](https://arxiv.org/abs/2204.01691) |
| 工具调用而不是写代码 | ALRM [arXiv:2601.19510](https://arxiv.org/pdf/2601.19510) |
| 执行后核对,失败交给模型 | Agentic AI for Robot Control: Flexible but still Fragile [arXiv:2602.13081](https://arxiv.org/html/2602.13081v1) |
