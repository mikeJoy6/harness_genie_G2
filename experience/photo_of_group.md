---
name: photo_of_group
when: 给一个人或一群人拍照
status: 真机成功(2026-11-13)
source: traces/20261113_011450.jsonl
---
## 原则
- 先让头对准要拍的人,再拍。广角相机(默认)视野大,适合拍人群。
- 照片只存在本机 photos/,不要说会发到别处。

## 做法
1. find_objects(query="person")
2. look_at_object(object_ids=[所有人的编号]):对准这几个人的中间;只有一个人就用 object_id
3. take_photo():默认广角

## 常见坑
- 一个人都没找到:告诉用户,不要对着空画面拍。
- 人太分散、对准中间后有人在画面外:照实说哪几个人可能没拍进去。

## 怎么确认成功
- take_photo 返回 ok 和照片路径,把路径告诉用户。
