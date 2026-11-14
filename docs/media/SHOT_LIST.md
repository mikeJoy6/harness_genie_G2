# 拍摄清单（README 缺的图和动图）

2026-11-14 整理。原来 `docs/media/` 里的 `demo.gif`、`scene.jpg`、`terminal.png`、`rviz.png`、`detection.jpg`、`hsv_pick.png` 都是“待补”占位图，而且有几张还是 HSV 时代的内容，已经删掉。两份 README 里对应的 `<img>` 先用 HTML 注释包起来（GitHub 上不显示）。拍好后放到下面写的文件名，**把 README.md 和 README.zh-CN.md 里对应的 `<!-- ... -->` 去掉**就能显示。

## 通用要求

- **画面里不要出现人脸、屏幕上的个人信息、`.env` 内容、API key。** 头部相机拍到的画面里有人的，不要用。
- 机器人视频用手机横拍，三脚架固定，侧前方 45°、略高于桌面，机器人上半身和桌面要同时在画面里。
- 运动段可以加速 2~4 倍，左上角加一行字说明“2× / 4×”，不要假装是实时的。
- 动图宽 960 px、12 fps，单个文件控制在 8 MB 以内（GitHub 上太大会加载很慢）。转换命令记在 devlog 2026.11.14。

## 1. `demo.gif` —— 首页主演示（最重要）

- **内容**：完整一条“低头找鼠标，拿起来”。
  1. 画面开头叠一行字幕，写出指令原文（中英各一行）；
  2. 转头看桌面（`look_at_object`）；
  3. 右手到鼠标上方（`approach_object`）；
  4. 张开夹爪；
  5. 下探（`touch_object`，最后几厘米明显变慢）；
  6. 闭合，抬起。
- **时长**：原始 1~2 分钟，加速后 15~25 秒。
- **可选**：右下角画中画放终端，能看到工具调用一条条出来。
- **备选**：如果“拿起来”在真机上还不稳，先拍“找手机，碰一下”（`find_objects` + `touch_object`），文件名不变。

## 2. `sam3.png` —— SAM 3 分割结果

- **内容**：桌上放 3~4 样东西（鼠标、手机、杯子、积木）。跑 `scripts/sam3_live.py` 或一次 `find_objects`，截图要能看到彩色掩码和编号（`mouse_1`、`phone_1`…）。
- **尺寸**：截原图，宽 640 或 960 都行。
- **注意**：头部相机朝桌面，画面里不要有人。

## 3. `terminal.gif` —— 终端里的工具调用

- **内容**：`python3 scripts/run_task.py --real "低头找鼠标,拿起来"` 从输入到结束的终端录屏，要能看到模型选工具 → 规划摘要 → 执行结果 → 最后的回答。
- **工具**：`peek` 或 `asciinema` + `agg` 转 gif。终端字体调大（16 pt 以上），窗口宽约 100 列。
- **时长**：等模型回复的部分剪短，总长 20~30 秒。
- **注意**：录之前清屏，别把 `.env`、环境变量和其他私人信息录进去。

## 4. `touch_chain.gif` —— RViz 里的 touch 两段链路（加分项）

- **内容**：RViz 打开 octomap（物体在点云里），机器人模型 + 末端坐标系。先看到手臂查着碰撞走到预接触点、停一下，再慢速贴上物体。
- **用途**：放在 README“Highlights”里 touch 那一条下面，也可以放进 [docs/touch_chain.md](../touch_chain.md)。
- **时长**：10~15 秒，可加速。

## 5. `pointcloud_cleanup.gif` —— 悬空点云被清掉（加分项）

- **内容**：RViz 只显示 octomap。用手在相机前晃一下，制造空中零星格，然后看它们在约 1 秒内消失，桌面、桌上物体保留。
- **用途**：放进 [docs/pointcloud_cleanup.md](../pointcloud_cleanup.md)。
- **注意**：只录 RViz 窗口，手不要出现在录屏里（RViz 里只是点云，没问题）。

## 6. `scene.jpg` —— 实验场景照片（可选）

- 一张普通照片：G2 站在桌前，桌上摆着要用的物体。光线充足，背景整洁，没有人。宽 1280 左右即可。

## 另外：`harness_overview.png` 过期了

全景图还画着 `colors.yaml`、`hsv_pick.py`、`locate_object`，和现在的代码不符，所以 README 没有引用它。要用的话得先改 [make_overview.py](make_overview.py) 重新生成。
