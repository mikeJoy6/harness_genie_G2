# sam3_click：SAM 3 交互分割演示

纯演示，不接 harness，机器人不会动（只订阅头部相机）。在实时画面上冻结一帧，然后点、拉框、打字，看 SAM 3 怎么补全物体。

| 文件 | 作用 | 用哪个 Python |
|---|---|---|
| `server.py` | 打开“点 / 框补全”的 SAM 3 服务（端口 8766）：先把一帧编码好存起来，之后每次点击只做补全 | `~/sam3_service/.venv/bin/python`（3.12） |
| `click_demo.py` | OpenCV 窗口：实时画面，空格冻结，鼠标和键盘操作 | 系统 `python3`（3.10 + ROS）；`--image` 时不需要 ROS |

## 启动

```bash
# 终端 1:常规 SAM 3 服务(8765)先停掉,省显存
cd ~/robot_harness
~/sam3_service/.venv/bin/python demos/sam3_click/server.py

# 终端 2
source ~/agi@g2_robot/env_g2.sh
cd ~/robot_harness
python3 demos/sam3_click/click_demo.py
python3 demos/sam3_click/click_demo.py --image traces/sam3/<某次>/color.jpg   # 不连机器人
python3 demos/sam3_click/click_demo.py --grid 32                              # 全分割撒点更密(小物件)
```

## 五种用法

| 键 | 模式 | 做什么 | 对应 SAM 3 的功能 |
|---|---|---|---|
| 1 | 点 | 左键点物体上，右键点“不是这个”；一个点时 Tab 切换 3 个候选（小部件 / 整个物体 / 更大一片）；n 留下这个物体 | SAM 1 式交互分割（`predict_inst`） |
| 2 | 框 | 拖框 → 补全框里那一个物体 | 同上，框提示 |
| 3 | 示例框 | 拖框 → 找出画面里所有和它同类的东西 | SAM 3 概念分割，示例框提示（`add_geometric_prompt`） |
| 4 | 文本 | 在终端输入英文名 → 找出所有这种东西 | SAM 3 概念分割，文本提示 |
| 5 | 全分割 | 网格撒点、每个点补全一个、去重，不要名字 | 交互分割 × 网格 |

空格冻结 / 解冻，c 清空，s 存图到 `photos/sam3_click/`（本机，已 gitignore），q / Esc 退出。

## 离线实测（2026-11-15，存好的头部 640×400 画面，RTX 4070 Laptop）

| 项目 | 结果 |
|---|---|
| 加载 | 8.5 s；显存常驻 3.6 GB（常规服务 3.4 GB，交互头多约 170 MB），峰值 4.7 GB |
| 编码一帧 | 60~570 ms（第一次慢） |
| 点 | 第一次约 2 s（预热），之后几十 ms；点鼠标中心补出的掩码和文本 `mouse` 的掩码重合度 0.96 |
| 框 | 约 10 ms |
| 文本 | 约 100~200 ms |
| 示例框 | 约 60 ms |
| 全分割（16 × 10 个点） | 约 0.7 s，分出 15 块；桌面、地面被面积上限（15%）挡掉；20 像素左右的小积木会漏（点距 40 像素），用 `--grid 32` |

**真机实时画面还没跑过**（机器人没开）。
