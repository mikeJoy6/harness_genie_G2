---
name: pick_up_object
when: 从桌上拿起 / 抓起一个东西
status: 真机成功 1 次(2026-11-14,鼠标)
source: traces/20261114_032417.jsonl
---
## 原则
- 夹爪从正上方竖直下去,张开的两指罩住物体两侧,再闭合。侧面去抓会把东西推走,还容易扫到桌面。
- 物体坐标是相机看到的表面(放在桌上的东西就是顶面)的中心,不是底。所以“下到物体”要越过顶面继续往下,
  直到夹爪尖碰到桌面才算到底 —— touch_object 在 gap_cm=0 时就是这么做的(六维力碰到就停),停下后自动退 2 厘米,给闭合留空间。

## 做法
1. find_objects 找到它,记下 object_id
2. approach_object(object_id, tool_axis="down"):夹爪朝下停在它上方(悬停高度不用填)
3. gripper(action="open")
4. touch_object(object_id, gap_cm=0):往下直到碰到,自动退 2 厘米。看结果里的 contact.felt
5. gripper(action="close")
6. move_arm_end(offset_cm=[0, 0, 10]):抬起

一共 6 次运动。

## 常见坑
- contact.felt 为 false:一路没碰到东西,夹爪可能还在半空。先告诉用户,不要直接闭合当作抓到了。
- 程序可能先让上身前倾(结果里有 body_assist),腰不会自己回去。之后要回原位用 go_to_pose。
- 夹爪不回报开合位置,gripper 返回 ok 只说明命令发出去了,不说明抓住了。

## 怎么确认成功
- 抬起后再 find_objects 找一次:抓住了,物体会跟着夹爪(坐标在夹爪尖附近、比原来高);原来的位置不再有它。
- 找不到、或者还在原来的位置:没抓住,如实告诉用户。
