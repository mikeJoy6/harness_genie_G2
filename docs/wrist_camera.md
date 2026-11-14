# 腕部相机：是什么、怎么确认对准、视野小时怎么找物体

> **2026-11-14 暂停：腕部相机的线丢了。** `harness.yaml` 里 `vision.wrist` 和 `take_photo` 的 `hand_right` 已注释掉，模型调不到腕部相机。代码和下面的设计都留着，线接回后先跑 `check_wrist_camera.py`，再去掉配置里的注释。

2026-11-14。代码在 [harness/perception/wrist_camera.py](../harness/perception/wrist_camera.py)（取一帧）和 [scripts/check_wrist_camera.py](../scripts/check_wrist_camera.py)（查话题）。**真机还没收到过一帧画面**，下面关于 RGB / RGBD 的判断要等脚本跑完确认。

## 1. 是 RGB 还是 RGBD

| 来源 | 说法 |
|---|---|
| G2 真机话题（2026-11-13） | 只看到 `/gdk/camera/hand_{left,right}_color`（`CompressedImage`），10 s 收不到帧；没看到腕部深度话题 |
| G01 GDK v1.5.0 指南（`~/a2d_sdk/`） | 腕部是 Intel RealSense **D405**（RGBD，近距离 7~50 cm 很准），彩色 848×480；v1.5.0 起才有深度话题 `/camera/hand_{left,right}_depth`。相机要在机器人端模式配置（`copilot.pbtxt` / `hybrid_deploy_develop.pbtxt` 的 `img_preproc`）里列出才会打开，`depth_enable: true` 才有深度 |
| URDF `G2_t2_crsB_ee.urdf` | 没有腕部相机的 link：相机相对夹爪装在哪不知道（没有手眼标定） |

**判断**：硬件大概率是 RGBD（如果 G2 沿用 D405），但现在 G2 只发布了彩色，而且彩色也没有帧。最可能的原因是机器人端相机配置没有打开腕部相机。下一步：

```bash
source ~/agi@g2_robot/env_g2.sh && cd ~/robot_harness
python3 scripts/check_wrist_camera.py --seconds 5
```

脚本会列出所有带 hand / wrist 的话题，逐个听，报告帧率、分辨率、编码，深度图给出有效像素比例，再查 TF 里有没有相机坐标系，最后给出“RGBD / 只有 RGB / 没有画面”的结论。第一帧存在 `photos/wrist_check/`。如果一帧都没有，就要找智元确认 G2 怎么打开腕部相机、能不能打开深度。

## 2. “对准了没有”怎么判断：不需要手眼标定

相机固定在手腕上，和夹爪一起动。所以“物体在两指之间”在腕部画面里永远对应**同一块像素区域**，不需要知道相机在哪：

1. **示教一次参考区域**：把一个物体正确地放在张开的两指之间，用 `approach_object` 那个固定悬停距离（12 cm）和下探到一半的距离各拍一张，记下物体掩码的中心和宽度。这就是“对准时物体应该在的地方”。
2. **检查**：分割出物体后，看掩码中心离参考中心多少像素、宽度合不合适。在容差内就算对准。
3. **纠偏**：像素偏差要换成手的移动量。不用标定，做一次“点动标定”就行：手沿夹爪 x、y 各移动 1 cm，看物体在画面里移动多少像素，得到一个 2×2 的换算矩阵。之后用 `move_arm_end` 小步挪，再拍再查，直到偏差在容差内（视觉伺服）。有深度时，还可以直接用深度把像素换成米。

## 3. 视野小、SAM 3 按名字认不出来时怎么办

腕部相机离物体很近，常见的情况是物体只露一部分，或者占满半个画面，按名字（`mouse`）分割可能失败。有几种办法：

| 办法 | 需要 | 优点 | 缺点 |
|---|---|---|---|
| A. 还用名字，降低阈值，选离参考区域最近的实例 | 什么都不用加 | 最简单，近距离大物体其实常常认得出 | 只露一角时容易失败 |
| B. 在“预期像素”放一个点让 SAM 补全 | SAM 3 服务开 `enable_inst_interactivity=True`（SAM 1 那种点提示，多占一些显存，要测） | 不需要名字；物体被切掉一半也能分出来 | 点落在桌面上就分出桌面，要用深度或掩码大小把关 |
| C. 只用深度（如果有）：去掉桌面平面，取参考区域里离夹爪最近、高出桌面的那一块 | 腕部深度 | 不用神经网络，快，D405 近距离很准，物体只露一角也能找到 | 不知道“是什么”；物体紧挨着别的东西时会连成一块 |
| D. 头部相机兜底 | 现有的 | 不用新东西 | 手一靠近就把物体挡住，正好是最需要的时候 |

“预期像素”不用另算：`approach_object` 把夹爪停在物体正上方固定高度，物体应该就在第 2 节示教的参考区域里。

**推荐组合**：
- **有深度**：用 C 判断“夹爪下面有没有东西、偏多少”，这是对准检查的主力。用 A 顺带确认是不是那个东西：认得出最好，认不出不算失败，因为物体是谁头部相机已经确认过了。
- **只有 RGB**：先 A；A 失败再用 B，在参考区域中心放点补全，再按掩码大小判断是不是把桌面分出来了。

A 现在就能测，不用写代码：

```bash
python3 scripts/sam3_live.py --topic /gdk/camera/hand_right_color "mouse"
```

## 4. 接进 harness 的顺序

1. 跑 `check_wrist_camera.py`，确认能收到帧、有没有深度（深度话题填进 `harness.yaml` 的 `vision.wrist.right.depth`）。
2. 用 `take_photo(camera="hand_right")` 或 `sam3_live.py --topic ...` 看几张画面，试 A。
3. 示教参考区域，写纯函数“掩码 / 深度块 → 偏差像素”，离线用录下的图测试。
4. 再加工具，比如 `check_grasp_alignment(arm)`：返回对准与否、偏差（像素和估计的厘米）。纠偏先交给模型用 `move_arm_end` 小步做，稳定后再收进 `pick_object`。
