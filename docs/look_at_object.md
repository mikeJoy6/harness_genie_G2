# 技能:按名字找物体 + 转头看它(SAM 3)

用户说“看向最近的手机”,模型调 `find_objects(query="phone")`,再调 `look_at_object(object_id="phone_1")`,机器人只转头,让手机落在画面中央。每次分割的画面和掩码都存进 `traces/sam3/`。

前提:[SAM 3 服务](sam3_service.md)开着。

## 流程

```text
用户:"看向最近的手机"
 │
 ▼ 模型 → find_objects(query="phone")                          不运动
   Gateway._find_objects            gateway.py:80   查 query 是英文;同一短语的旧编号作废
   RosBackend.find_objects          ros.py:147
     _grab_frame                    ros.py:133      一帧:彩色 + 深度(mm)+ base_link→head_link3
     Sam3Client.segment(bgr, ["phone"])             → 每个实例的掩码
     Rgbd.points_base                               深度 → base_link 3D 点
     objects_from_masks             sam3_objects.py:29   掩码内有效深度 → 中位点;按离相机由近到远编号 phone_1、phone_2…
     save_segmentation              sam3_objects.py:70   → traces/sam3/<时间>_phone/
   Gateway 存表 self.objects["phone_1"] = {xyz_m, time, label}     坐标只留在 gateway
   ← 给模型:编号、坐标、距离、离画面中心多少像素(第一个最近)
 │
 ▼ 模型 → look_at_object(object_id="phone_1")                  运动(默认自动执行)
   Gateway._look_at_object          gateway.py:102  查编号存在、没过期(30 s)、运动次数没用完
   RosBackend.head_angles_to        ros.py:176
     HeadAim.solve                  head_aim.py:51  URDF 链 base_link→head_link3 + 相机外参;
                                                    只改 yaw(idx11)、pitch(idx13),2×2 高斯-牛顿,夹在限位里
   查每个头关节单次 ≤ 90°
   _plan_and_run_joints(..., ["head"])               和 move_joints 同一条路:URDF 限位 → WholeBody 规划
                                                    (整机碰撞检查,会撞就绕)→ 只规划模式停 → 执行(`--confirm` 时先按 y)
   _check_centered                  gateway.py:138  再分割一次(存成 _after),按 3D 位置找回同一个物体,
                                                    返回离画面中心的像素偏差
```

## 夹爪指向物体:`point_at_object`

“让右夹爪指向我的手” → `find_objects(hand)` → `point_at_object(object_id="hand_1", arm="right")`。
末端位置不动,工具轴(末端 Z)转到“末端 → 物体”的方向,roll 锁住不自转;原地转不到就复用“后撤着转”(`tool_axis_retreat`,见 [goal_pose.md](goal_pose.md))。
末端离物体 < `point_min_distance_m`(0.15 m)拒绝,防止戳到人。执行后报 `pointing_error_deg`(工具轴和“末端→物体”的夹角)。
录的真机帧上 SAM 3 `hand` 找到画面里那个人的两只手(0.79、0.74)。

## 夹爪尖碰到物体:`touch_object`(击掌、碰拳)

“和我击掌” → `find_objects(hand)` → `touch_object(object_id="hand_1", arm="right")`。
夹爪朝“末端 → 物体”(`tool_axis`,roll 锁住),末端连杆停在物体前 `夹爪长度 0.128 m + gap_cm(默认 2)` 处,不用模型算偏移。
物体本身在点云里,规划会走接触兜底:先避障到附近,最后一小段 0.03 m/s 不查外界碰撞贴过去。结果报 `tip_to_object_m`(夹爪尖离物体)。
配置 `motion.touch`(tool_length_m、default_gap_cm、max_gap_cm)。

## 拍照:`take_photo`

“给我们这群人拍照” → `find_objects(person)` → `look_at_object(object_ids=[所有人])`(对准这些人的中间)→ `take_photo()`。
从相机话题取一帧 JPEG **原样**存到 `photos/<时间>_<相机>.jpg`(不重新编码;已 gitignore,照片里有人)。不算运动,只规划模式下也照拍。

| camera | 话题 | 分辨率 | 用途 |
|---|---|---|---|
| `head_wide`(默认) | `/gdk/camera/head_stereo_left` | 1920x1536,广角 | 拍人、拍全景 |
| `head` | `/gdk/camera/head_color` | 640x400 | 和深度对齐的那一路 |

左右手相机话题 `/gdk/camera/hand_{left,right}_color` 在列表里,但 2026-11-13 实测收不到帧,暂不开放。配置 `vision.photo`。

## 手臂去找到的物体上方

同一个编号也能给手臂用:`approach_object(object_id="phone_1", arm="right", standoff_cm=30)`。和按颜色定位走同一套检查(悬停高度、工作范围、移动距离、防穿桌)和规划(直线 → 绕障),只是物体坐标从编号表取。

## 为什么这样设计

- **模型只传编号**,坐标在 gateway 里([物体编号表](object_handles.md)的第一次落地)。编了不存在的编号会被拒,并告诉它有哪些。
- **“最近的”由程序排序**,模型只要选第一个,不用自己比较坐标。距离是到相机的直线距离。
- **只动头**:`look_at_object` 只生成头部 3 个关节的目标,歪头关节(idx12)保持当前角度,腰和手臂不动。头部转动仍走 `WholeBody` 的碰撞检查。
- **算角度用正运动学,不用像素比例**:相机不在头关节的转轴上(往前约 11 cm),而且零位时相机就往下看 25°。所以用 URDF 正运动学 + 外参迭代算,能考虑腰的当前姿态。
- **头转不到时如实说**:URDF 限位 yaw ±44.7°、pitch −1°~30°。超出时停在限位附近(留 0.03 rad),返回 `head_at_limit: true` 和预计还差多少度。
- **转完再看一眼**:结果里的 `check_after.offset_from_center_px` 是实测的,不是算出来的;两张图都在日志里可以对比。

## 日志:`traces/sam3/<时间>_<短语>[_after]/`

| 文件 | 内容 |
|---|---|
| `color.jpg` | 原始彩色画面 |
| `overlay.jpg` | 掩码 + 编号 + 距离 + 画面中心十字 |
| `mask_<编号>.png` | 每个物体的掩码(0/255) |
| `depth.png` | 深度,uint16 毫米,离线可以重算 3D |
| `result.json` | 短语、阈值、耗时、相机位置、`base_link→head_link3`、当时的头部关节角、每个物体的编号/分数/坐标/距离/像素/框、被丢掉的原因 |

`traces/sam3/` 已 gitignore(画面里可能有人)。工具结果里的 `log_dir` 指向这次的文件夹,`traces/<时间>.jsonl` 里也能查到。

## 配置(`configs/harness.yaml`)

| 键 | 默认 | 作用 |
|---|---|---|
| `vision.sam3.url` | `http://127.0.0.1:8765` | 服务地址 |
| `vision.sam3.threshold` | 0.5 | SAM 3 置信度 |
| `vision.sam3.timeout_s` | 10 | 一次分割最多等多久 |
| `vision.sam3.min_valid_depth_pixels` | 15 | 掩码里有深度的像素少于这个就不算 3D |
| `motion.detection_max_age_s` | 30 | 编号多久后作废 |
| `motion.look_at_match_m` | 0.15 | 转头后再分割,离原位置这么近才算同一个物体 |
| `motion.joint_speed_rad_s.head` | 0.30 | 转头速度 |
| `motion.max_joint_delta_deg.head` | 90 | 单次每个头关节最多转多少 |

## 验证

- `tests/test_sam3_look.py` 18 个:假机器人上找物体、排序、中文短语被拒、服务没开、重新找时旧编号作废、只动头、只规划不动、用户拒绝、编号不存在/过期、已经居中、整条“看向最近的手机”的模型对话;掩码 → 物体、日志文件齐全;`HeadAim` 用真 URDF 算到残差 < 0.1°、头转不到时报限位。
- 录的真机帧离线跑通 `find_objects` 的整条感知链(真 SAM 3 + 真深度 + 录的头部位姿):`phone` 找到画面里人手上的手机(1.75 m),积木、笔在桌面高度 z≈0.74~0.78 m;3D 点反投影回画面和掩码中心差 < 1 像素。
- **真机转头还没跑过**。先 `--plan-only`。
