---
name: high_five
when: 和人击掌、碰拳、摸一下人的手
status: 真机做过多次,最近一次(2026-11-15)碰到手的位置但没感到接触力
source: traces/20261115_232041.jsonl;devlog 2026.11.13
---
## 原则
- 这是用户要求的接触:夹爪从正面慢慢靠近人手是对的。touch_object 会让夹爪朝向手,最后一段慢速、
  碰到就停(六维力),所以可以直接用。
- 人手会动:find_objects 之后尽快做,不要中间插别的事。
- 手里拿着东西时不要击掌:先问用户要不要先放下。

## 做法
1. find_objects(query="hand"):有几只手时,选举起来的(z 高)、离机器人近的那只
2. touch_object(hand_x, gap_cm=0):碰到就停。用户说“离我手 N 厘米”就填 gap_cm=N
3. move_arm_end(offset_cm=[-10, 0, 0]):往回收一点,别一直贴着人手

## 常见坑
- reachable_by 为空也可以直接试,程序会前倾帮忙;够不着就如实告诉用户把手伸近一点。
- gap_cm=0 时夹爪会越过手的位置再往前最多 5 厘米,碰到(力 > 5N)才停。2026-11-15 走完全程也没感到力
  (contact.felt 为 false):手可能已经挪开,或者人手软、力不到 5N。照实告诉用户“可能没拍上”,请他把手伸近再试。

## 怎么确认成功
- gap_cm=0 时:touch_object 结果里 contact.felt 为 true。gap_cm 大于 0 时:tip_to_object_m 接近 gap_cm。
