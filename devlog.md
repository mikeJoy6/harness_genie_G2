# robot_harness 开发日志

# 2026.11.09

## 去掉对 agi@g2_robot 代码的依赖

问题:harness 运行时 import 了 `agi@g2_robot` 的三个包(运控、感知、`task_board_pose`),去 `look` 位姿还要另起 `processor`。

现在运行时唯一的外部进程是 `hardware.launch.py end_effector:=true use_head_cloud:=false`。

| 新位置 | 复制自(`agi@g2_robot` 提交 `ecdfb5d`) | 改动 |
|---|---|---|
| `harness/motion/` 运控 | `genie_robot_agibot_g2_motion`:`arm_motion`、`kin`、`algorithms`、`ik_lab` 的 4 个文件 | `arm_motion.py` 两行 import |
| `harness/motion/planners/` | 同上,只拷 `via_then_rrt` 及依赖(7 个文件) | 无 |
| `harness/motion/` 工具函数 | `genie_robot_agibot_g2_perception`:`octomap_util`、`voxel_util` | 无 |
| `harness/motion/` 全身位姿 | `task_board_pose`:`whole_body`、`joints`、`joint_detour` | 无 |
| `harness/perception/` | `task_board_pose`:`rgbd_source`、`scene3d`、`calib`、`sensor_node` | 无 |
| `configs/poses.yaml` | `task_board_pose/config/poses.yaml` | 无 |
| `scripts/motion_cli.py` | `genie_robot_agibot_g2_motion` 的 `motion_cli` | import |
| `tests/test_motion.py` | `test_algorithms.py` | import;没有 URDF 时跳过 |

绕障规划器选 `via_then_rrt`:原项目 `planner_bench` 3 组留出题 183/196,9 个规划器里最好,规划中位 0.58s,也是原项目默认。

新增:

- `scripts/goto_pose.py`:替代 `processor` 的 `goto_pose`。打印每组最大关节变化,输入 `y` 才动;逐段问 MoveIt 自碰撞,会撞时经录好位姿中转或关节 RRT。
- 靠近时先试直线,失败再用 `via_then_rrt` 绕障。
- 防穿桌检查从“起点高度”改为“整条路径上末端最低点”,用正运动学逐点算;绕障路径不是直线,只看起点不够。配置项改名 `min_path_above_object_m`。

验证:

- 即使 source 了 `agi@g2_robot/install`,harness 加载的模块里没有一个来自 `agi@g2_robot`。
- 测试:不带 ROS 27 个通过、1 个跳过;带 ROS 43 个通过,含原项目的“绕柱”绕障测试。
- 隔离域仿真(单进程模拟关节、自碰撞服务、4 个 JTC、头部相机):`goto_pose look` 按 `n` 不发轨迹,按 `y` 4 组到位误差 0°;
  直线靠近误差 0mm;强制直线失败后切到 `via_then_rrt`,规划执行成功,路径最低点 0.701m。
- 修了 `goto_pose.py` 退出时崩溃(`terminate called without an active exception`):先停 spin 线程再销毁节点。

注意:`harness/motion/`、`harness/perception/` 是副本,原项目修 bug 后要手动同步(方法见 `harness/motion/README.md`)。

## 最小 harness 上真机

目标:一句自然语言让 G2 左臂末端移到头部相机看到的物体正上方。

### 结构

| 层 | 文件 | 职责 |
|---|---|---|
| 循环 | `harness/loop.py` | 问模型 → 执行工具 → 结果交还模型,`max_steps=8` |
| 模型 | `harness/llm.py` | DeepSeek,`deepseek-flash`,关闭思考模式 |
| 关卡 | `harness/gateway.py` | 目标点计算、全部安全检查、人工确认 |
| 执行 | `harness/backends/ros.py` | 头部 RGBD、TF 发布、`ArmMotion` |

关键决定:

- **模型不给坐标**。`approach_object` 只有 `standoff_cm`,目标点 = 最近一次定位结果 + 正上方偏移。
- **工具调用而不是生成代码**:每步都能观察结果,出错只影响一步。
- **运动用 `ArmMotion.plan_cartesian`**:直线、保持当前姿态、构型连续,轨迹按比例放慢到 5cm/s。
- **不加载点云**(`use_head_cloud:=false`),只有自碰撞检查,配合软桌和有人值守。
- `llm.py` 只用标准库 HTTP,避免往 ROS 的 Python 环境里装 `openai` 及其依赖。
- DeepSeek 思考模式下带工具的请求必须回传 `reasoning_content`,否则 400;`llm.py` 已原样回传。

### 安全检查(gateway)

先定位才能动 · 定位 30s 作废 · 定位失败清除旧目标 · 每条指令最多动一次 · 悬停 18~40cm(工具长 0.128m)·
工作空间 · 单次移动 ≤ 0.70m · 起点比物体高 ≥ 13cm(防直线穿桌)· 真机运动前输入 `y` · `--plan-only` 绝不运动。

### 验证

- 离线:27 个测试(假模型 + 假机器人 + 合成图像)。
- 仿真:隔离域(`ROS_DOMAIN_ID=77`、localhost)单进程模拟关节反馈、自碰撞服务、JTC、头部相机,
  本地假 DeepSeek 服务校验请求格式和 `reasoning_content` 回传。从 `look` 位姿出发:定位误差 1~4mm,
  规划 17.4cm 直线,按 `n` 不发轨迹,按 `y` 发 1 条并到位。
- 真机只读:实际头部画面定位青椒、红积木,确认绿框位置正确。

## 问题记录

| 现象 | 原因 | 处理 |
|---|---|---|
| `UnicodeEncodeError: 'latin-1'` | `DEEPSEEK_API_KEY` 是示例占位符“你的key” | key 改写在 `.env`(优先于环境变量,权限 600);非 ASCII key 直接报清楚的错 |
| 规划失败,位置差 35.9mm | 积木在 (0.806, -0.044),超出左臂可达范围;全姿态、只要位置的多种子 IK 都无解 | 把物体挪近;离线算了当前腰部姿态下的可达范围图,x 0.40~0.75、y 0~0.40 基本可达 |
| “抵达绿色青椒”被模型拒绝 | 工具只认红色 | 改为按颜色定位,颜色表 `configs/colors.yaml`,新增 `scripts/hsv_pick.py` 点选取色 |
| “画面里有 2 个红色物体” | 多候选时直接拒绝 | 默认选离左臂末端最近的;各帧选到不同物体时报“不稳定”拒绝;调试图黄框标出全部候选 |
| `look` 位姿下会被防穿桌检查拒绝 | 末端高 0.77m,与桌面上的物体同高 | 先用 `motion_cli` 抬高 20cm,已离线确认抬高后可达 |

## 实测数据

| 物体 | `base_link` 坐标 (m) | 深度 (m) | 可见宽度 (m) |
|---|---|---|---|
| 红积木(第一次,够不到) | (0.806, -0.044, 0.766) | 0.785 | 0.036 |
| 青椒 | (0.761, 0.033, 0.768) | 0.629 | 0.144 |
| 红积木(挪近后,2 个候选中选最近) | (0.790, 0.097, 0.768) | 0.772 | 0.030 |

左臂末端在 `look` 位姿:(0.588, 0.423, 0.768);抬高后的实测起点:(0.677, 0.392, 1.100)。

真机执行“左手抵达红色积木上空”(`traces/20261109_015006.jsonl`、`015123.jsonl`):

| 项目 | 结果 |
|---|---|
| 目标点 | (0.790, 0.097, 1.017),积木上方 25cm |
| 规划 | 积分笛卡尔直线 32.6cm,57 个轨迹点,8.5s |
| 执行 | JTC 成功 8.9s,实测停稳 |
| 到位误差 | 0.1mm(两次相同) |
| 模型行为 | 两次都按 定位 → 读末端 → 靠近 的顺序调用,一次把前两个工具并行调用 |

## 待办

- [x] 真机按 `y` 完整执行一次靠近(2 次,到位误差 0.1mm)
- [ ] 抬手工具 `lift_left_end`(练习见 `docs/harness_guide.md` 第 17 章)
- [ ] 补 README 的图片和动图(`docs/media/`)
- [ ] 桌面作为碰撞体加入 MoveIt(绕障规划才能绕开桌子),或开启点云避障
- [ ] 腰部 / 底盘工具,够不到时让模型先调整身体
- [ ] 吸盘抓取(`suction_control`)
- [ ] `scenarios/` 模型评测用例,`fixtures/` 真机录制数据

# 2026.11.08

## 方案与目录

- 对 harness 的定位:模型外面那层程序,负责工具描述、执行、安全、状态和记录。
- 选择“工具调用”方式(参考 ALRM 的 Tool-as-Policy),不让模型生成机器人代码。
- 底座复用 `agi@g2_robot`:运控、感知、`task_board_pose` 任务框架都不拷贝,运行时直接 import 已安装的包。
- 建立 10 个目录,每个目录写 README,说明作用和联动。
- 本机 RTX 4070 Laptop 8GB 显存跑不动 Qwen3.8,先用 API,接口保持 OpenAI 兼容格式,方便以后换本地模型。
