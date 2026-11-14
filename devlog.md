# robot_harness 开发日志

设计文档在 [docs/](docs/index.md),这里只记每天做了什么、遇到什么问题。

# 2026.11.14

## 准备上 GitHub:英文 README 做默认,中文改名,拍摄清单

- `README.md` → [README.zh-CN.md](README.zh-CN.md)(`git mv`),内容顺手更新到现状:HSV 颜色定位删掉、改成 SAM 3 + 物体编号;工具表补 `find_objects` / `touch_object` / `gripper`;安全表按现在的配置(悬停 3~30cm 默认 12cm、物体 120s 作废、每条指令最多动 8 次、只有 touch 最后一段不查点云)。
- 新的英文 [README.md](README.md) 作为 GitHub 默认页,两份顶部互相链接。加了 Quick start(离线测试、假机器人、真机只规划)。
- `docs/media/` 里的 6 张“待补”占位图删掉(`demo.gif`、`scene.jpg`、`terminal.png`、`rviz.png`、`detection.jpg`、`hsv_pick.png`)。README 里的图先用 HTML 注释包着,要拍什么写在 [docs/media/SHOT_LIST.md](docs/media/SHOT_LIST.md),拍好后去掉注释。
- `harness_overview.png` 还画着 HSV,已在 [docs/index.md](docs/index.md) 标“已过期”,README 不引用。

```bash
# 手机视频 → README 用的动图(宽 960、12fps;加速 3 倍就把 setpts=PTS/3 加进去)
ffmpeg -i demo.mp4 -vf "setpts=PTS/3,fps=12,scale=960:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse" -loop 0 docs/media/demo.gif
ls -lh docs/media/demo.gif                               # 控制在 8MB 以内,太大就改 scale=720 或 fps=10

# 终端录屏 → 动图
asciinema rec /tmp/terminal.cast                         # 录完 Ctrl-D
agg --font-size 16 /tmp/terminal.cast docs/media/terminal.gif
```

# 2026.11.13

## 框架梳理:只靠 SAM 3、touch 固定两段、点云悬空块、兜底链改成步骤列表

起因:先做了一次框架检查(颜色阈值散在 3 处且没标定过、接触兜底对所有手臂运动生效且不查自碰撞、“拿起来”6 次运动超过上限 5、物体 30s 过期、两个执行器“起点在接触里”规则不一样、提示词自相矛盾)。用户定:① 纯靠 SAM 3,删 HSV;② 不查点云只给 touch_object,要一条明确链路,第一段能去就一定规划得出来;③ 空中零星点云不管在不在视野里都要及时删;④ 其余建议照做。

**① 删 HSV 颜色定位整套**:`harness/colors.py`、`backends/color_object.py`、`scripts/hsv_pick.py`、`configs/colors.yaml` 删除;工具 `locate_object`、`get_left_end_pose` 删除;gateway 的单槽位 `self.target` 删除,物体只能用 `find_objects` 的编号([`_object`](harness/gateway.py#L153) 统一查编号 + 过期);`approach_object` 的 `object_id` 改必填;`vision:` 下颜色专用参数删掉。README 们只删了过时的行。

**② 运动链路**([docs/touch_chain.md](docs/touch_chain.md)):
- [`plan_arm_to`](harness/backends/ros.py#L240) 改成按顺序试的步骤列表(`_step_orientations` → `_step_tilt_here` → `_step_tilt_retreat` → `_step_body_assist`),参数打包成 [`ArmGoal`](harness/backends/ros.py#L42);**接触兜底删掉**,approach / move_arm_end / point 规划失败就是失败。
- 新 [`plan_touch`](harness/backends/ros.py#L382):在接触点定姿态和关节角 → 从接触点沿直线反着积分到预接触点(同一构型分支)→ 验预接触点不撞 → 第一段直线 / via_then_rrt / **关节空间直接去验过的关节角** → 最后一段 [`_execute_touch`](harness/backends/ros.py#L627) 不查点云。
- “不查点云”= [`ArmMotion.valid(ignore_cloud=True)`](harness/motion/arm_motion.py#L373):只放过 `<octomap>` 的接触,**自碰撞和碰撞体照查**(原来 `check_collision=False` 什么都不查)。
- 离线踩坑:最初在预接触点解 IK,真实 URDF 下解到关节 1 −163° 的分支,而接触点在 −7°,37 个候选全是“最后一段走不通”;改成从接触点反着积分后立刻有解。
- [WholeBody](harness/motion/whole_body.py#L122) 也加“脱离”:起点就有、涉及要动部件的接触,离起点 0.10 rad 内不算(夹着鼠标也能回初始位姿)。

**③ 点云**([docs/pointcloud_cleanup.md](docs/pointcloud_cleanup.md)):根因是 `shadow_cloud` 把每个漏网点往后补 25cm(~8 格),sweeper 只清 <4 格 → 一个假点就是一根永远清不掉的条。`octomap_sweeper.py` 新增悬空块规则(<40 格且最低处高于 0.15m 就清)、相机动了按新位置接着清、0.5s 一轮、10Hz。改在 `~/agi@g2_robot/src/genie_robot_agibot_g2_perception/`(+ `genie_moveit_config/config/sensors_3d.yaml`),已同步到 `~/ai_assistant/src/g2_head_pointcloud/`。

**④ 其余**:`max_moves_per_command` 5 → 8;`detection_max_age_s` 30 → 120;系统提示删掉“先 locate_object”那句、写明只有右手有夹爪、默认右手;`approach_object` 默认右臂;工具说明去掉“需要用户确认”;gateway 文件头说明更新;结果说明合并成 [`_plan_notes`](harness/gateway.py#L57)。

测试:`test_objects.py` 改成按 find_objects 编号;`test_ros_backend_logic.py` 重写成 touch 链路(改用真机的 `G2_t2_crsB_ee.urdf`);新 `test_octomap_sweeper_pick.py`;`test_tcp_and_escape.py` 加 `ignore_cloud`。结果 127 过、1 败(`test_arm_target_outside_workspace_refused`,是 harness.yaml 被放宽的工作范围,不是代码问题)。**真机全部没跑。**

```bash
# 离线测试
cd ~/robot_harness && source /opt/ros/humble/setup.bash
python3 -m pytest tests -q
python3 -m pytest tests/test_ros_backend_logic.py tests/test_octomap_sweeper_pick.py -q

# 点云改动要重新编译(install 是拷贝,不是软链),然后人工重启 hardware.launch.py
cd ~/agi@g2_robot && source env_g2.sh
colcon build --packages-select genie_robot_agibot_g2_perception genie_moveit_config
# 重启后看 sweeper 日志:“小块 N 格、悬空块 M 格:清 …”
ros2 node list | grep octomap_sweeper

# 真机试 touch 链路(先只规划,看 summary 里的“接近方向 / 预接触点 / 不查点云直线”)
cd ~/robot_harness && source ~/agi@g2_robot/env_g2.sh && source ~/.cache/agibot/app/env.sh
python3 scripts/run_task.py --real --plan-only "低头找鼠标,拿起来"
python3 scripts/run_task.py --real "低头找鼠标,拿起来"
```

## 手臂规划末端改到 TCP(夹爪尖)+ 抬起时允许“脱离”接触

trace `traces/20261113_020047.jsonl`:“低头找鼠标,拿起来”到上方、开夹爪、`touch_object(gap 0)` 下到鼠标、闭合都成功,但**没抬起来**——模型合上夹爪就停下问“要不要抬”。就算去抬也会失败:夹爪贴着鼠标(鼠标在点云里),起点就“撞”,手臂直线规划在 0% 处拒绝(随后 `go_to_pose` 也是 `0% 处撞:<octomap>↔arm_r_end_link`,靠关节 RRT 才出来)。
用户还指出:目标点按法兰 `arm_r_end_link` 算,夹爪长度只在 touch_object 里用常数凑,开夹爪会挨着桌子,闭合容易顶桌面触发保护。

查 URDF:现在机器人加载的 `/robot_description`(end_effector:=true → `G2_t2_crsB_ee.urdf`)**已经有** `tcp_r_link`(法兰沿 z 0.128m,注释写的是“末端长方体(吸盘)端面”)。用户说加载错了 URDF,有正确的那份;下面的改法对哪份都适用,只要 TCP 叫 `tcp_l_link / tcp_r_link`。

改动:

- [kin.py](harness/motion/kin.py) `arm_tip()`:URDF 里有 `tcp_*_link` 就把手臂链末端设到 TCP,否则法兰。所有手臂目标、`get_robot_state` 的末端(`link` 字段写实际用的哪个)、路径高度检查、舒适姿态的 c(腕心→末端)都变成“夹爪尖”。
- [harness.yaml](configs/harness.yaml) 按夹爪尖重算:`standoff_default_m` 0.25→0.12、`standoff_min_m` 0.18→0.03、`standoff_max_m` 0.40→0.30、`min_path_above_object_m` 0.13→0.02、`touch.tool_length_m` 0.128→0、`arm_reach_m` 0.72→0.80。工具说明 standoff 写明“夹爪尖,3~30,默认 12”。
- [arm_motion.py](harness/motion/arm_motion.py) `path_clear`:起点就撞时允许“脱离”——末端离起点 `ESCAPE_M` 0.06m 以内的碰撞不算,不撞了以后不许再撞,走出 6cm 还撞就失败(robot_harness 加的)。
- [系统提示](harness/loop.py):“拿起来”= 朝下到上方 → 开夹爪 → touch_object → 闭合 → move_arm_end 上抬 10cm,整串做完不要中途问。
- 测试:新 [tests/test_tcp_and_escape.py](tests/test_tcp_and_escape.py)(ee URDF 末端 = tcp_r_link、TCP = 法兰 + 0.128·z;脱离 4 种情况)。原限位下全过。**真机没跑**。
- 还没解决:夹爪尖下到物体“上表面”(gap 0)闭合是夹不到东西的——要下到物体中部、又不碰桌面,需要知道物体高度(桌面高度可以从掩码周围一圈的深度估)。留给抓取技能。

命令:

```bash
# 确认机器人加载的 URDF 里 TCP 的名字(换正确 URDF 后再看一次)
source ~/agi@g2_robot/env_g2.sh
ros2 topic echo --once /robot_description | grep -o 'link name="tcp_[a-z_]*"'
cd ~/robot_harness
source /opt/ros/humble/setup.bash && python3 -m pytest tests/test_tcp_and_escape.py -q
python3 scripts/run_task.py --real "低头找鼠标,拿起来"
```

## bug:接触兜底里又套了“动腰” → `KeyError: 'plan'`,而且腰已经动了

trace `traces/20261113_015556.jsonl`:动腰 5° + 到鼠标上方 25 cm 成功、夹爪打开;之后往下(`move_arm_end -22cm`、`touch_object`、`move_arm_end -15cm`)全部 `KeyError: 'plan'`。

原因:往下到鼠标要走接触兜底;[`_plan_contact`](harness/backends/ros.py) 找接近点时调 `plan_arm_to(..., allow_contact=False)`,只关了“再套接触兜底”,**没关动腰**。接近点够不着时内层返回了“先动腰再伸手”的两段式计划(没有 `plan` 字段),外层读 `first["plan"]` 就 KeyError。`touch_object` 那次在执行阶段出错:**腰已经又前倾了 5°**(j3 −49.6° → −39.7°,末端从 0.97 降到 0.91 m),但模型只看到“内部错误”。

修:

- 内层找接近点 `allow_body=False`。
- [`_execute_body_then_arm`](harness/backends/ros.py):腰动完之后的手臂规划 / 执行包在 try 里,出错也返回“腰已经前倾 X°(机器人动了),之后手臂出错:…”,不再只是一个异常。
- 新 [tests/test_ros_backend_logic.py](tests/test_ros_backend_logic.py):真实 RosBackend 规划逻辑 + 替身手臂 / 腰 + 真实 IK,用这次的腰、右臂、鼠标坐标:① 上方 25 cm(动腰)→ 往下到夹爪尖离鼠标 2 cm(鼠标周围 10 cm 是点云,走接触兜底),到位 < 1 cm、夹爪离竖直 < 16°;② 接触兜底里所有内层规划都 `allow_body=False`。把修复临时撤掉,② 失败(能抓住这个 bug);恢复后都过。
- 全部测试:原限位下全过。

命令:

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_harness
python3 -m pytest tests/test_ros_backend_logic.py -q
# 真机(SAM 3 服务开着)
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
python3 scripts/run_task.py --real "低头找鼠标,拿起来"
```

## “低头找鼠标,拿起来”:手臂够不着时自动动腰

trace `traces/20261113_015236.jsonl`:找到鼠标、转头对准都成功;`approach_object(right, tool_axis=down)` 失败,鼠标离右肩 0.77 m(伸直 0.70 m)。模型建议“往前走一步”(没有底盘工具)。

- 新 [`body_assist_search`](harness/motion/goal_pose.py) + [`RosBackend._plan_body_assist` / `_execute_body_then_arm`](harness/backends/ros.py):规划全失败后,找改动最小的前倾(j3)/ 转腰(j5),只看 IK;先动腰(整机碰撞检查),再从新姿态规划手臂。配置 `motion.body_assist`(enabled、max_lean_deg 30、max_turn_deg 20、speed_rad_s 0.10)。[gateway](harness/gateway.py) 结果带 `body_assist`。
- bug:接触兜底的预检查在 tool_axis 时只要 tilt_search 有结果就算“够得着”,这次 50° 也算,白试 6 个接近点 → 改成要求离目标方向 ≤ `tool_axis_tolerance_deg`。
- 提示:`reach_note` 和系统提示写明“reachable_by 空也先直接调用,程序会自动前倾 / 转腰;没有底盘工具,不要建议走过去”。
- 验证(替身 + 真实 IK,日志里的腰和右臂):前倾 10° 就够(0.2 s);执行后末端差 3 mm、夹爪离竖直 5.4°。测试 `test_body_assist_is_reported`;原限位下全过。**真机没跑**。
- 还缺:“拿起来”(抓取:预抓位 → 开夹爪 → 下降 → 合 → 抬起)。

命令:

```bash
# 终端 1
cd ~/sam3_service && .venv/bin/python server.py
# 终端 2
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real --plan-only "右臂移到鼠标上方,夹爪朝下"
python3 scripts/run_task.py --real "右臂移到鼠标上方,夹爪朝下"
# 关掉自动动腰:configs/harness.yaml 里 motion.body_assist.enabled: false
```

## GDK 日志不再堆在项目根目录

根目录堆了十几个 `python3.10.INFO`、`python3.10.log.INFO.<时间>.<pid>.0`:每次连 GDK(夹爪)glog 都往**当前目录**写。它们本来就被 `.gitignore` 的 `*.INFO`、`*.log.INFO.*` 忽略,只是乱。

- 实测 `GLOG_log_dir` 能改写位置(`GLOG_minloglevel` 关不掉 E 日志,但目录可以改)。
- [gripper_gdk.py](harness/backends/gripper_gdk.py) 在 `import agibot_gdk` 前 `os.environ.setdefault("GLOG_log_dir", <项目>/logs/gdk)`;`.gitignore` 加 `logs/`;已有的日志挪进 `logs/gdk/`。
- 真机验证:`gripper_cli.py status` 后根目录 0 个新日志,`logs/gdk/` 多 1 个。

命令:

```bash
ls ~/robot_harness/logs/gdk | tail
rm -f ~/robot_harness/logs/gdk/*      # 想清就清
```

## 拍照:`take_photo`

trace `traces/20261113_011450.jsonl`:“给我们这群人拍照” → 模型说没有拍照功能。

- 查相机:`/gdk/camera/head_color` 640x400 JPEG;`/gdk/camera/head_stereo_left|right` **1920x1536** JPEG、广角、彩色;`/gdk/camera/hand_{left,right}_color` 话题在但 10 s 收不到帧。
- 新工具 `take_photo(camera)`([gateway `_take_photo`](harness/gateway.py) → [`RosBackend.take_photo`](harness/backends/ros.py)):临时订阅一帧 `CompressedImage`,JPEG 原样写 `photos/<时间>_<相机>.jpg`;默认 `head_wide`。不算运动。`photos/` 已 gitignore。配置 [harness.yaml](configs/harness.yaml) `vision.photo`。
- `look_at_object` 加 `object_ids`:对准几个物体的中间(拍一群人)。系统提示:拍一群人 = find_objects(person) → look_at_object(object_ids) → take_photo。
- 真机只读验证:广角 1920x1536 用时 0.49 s,彩色 640x400 0.02 s。测试 `test_take_photo_default_wide`、`test_look_at_group_center_then_photo`。

命令:

```bash
# 终端 1
cd ~/sam3_service && .venv/bin/python server.py
# 终端 2
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real "给我们这群人拍照"
ls -t photos | head
xdg-open photos/$(ls -t photos | head -1)
```

## “指着粉色物体,手离它 1 厘米”:模型把默认值当成下限

trace `traces/20261113_010229.jsonl`(英文指令):① 先 `locate_object("purple")`(颜色表里没有粉色,硬凑成紫色,0 帧看到);② `find_objects(pink object)` 成功、`reachable_by: [right]`;③ `point_at_object` 成功;④ 模型说“touch_object 最小间隙 2 cm,1 cm 做不到”“超出舒适可达范围”,停下问用户。

都是工具说明的问题,机器人没有失败:`gap_cm` 能填 0~10,说明写的“不填 = 2;只有用户要求时才填”被读成了下限;`reachable_by` 写着 right,它却自己猜够不着。

改:[tools/objects.py](harness/tools/objects.py) `gap_cm` 写明“0~10,用户说了距离就照填(1 厘米填 1、贴上去填 0)”;`touch_object` 用途加“停在它跟前几厘米”;`locate_object` 标成备用(SAM 3 没开时才用,颜色不在列表里不要硬凑)。[loop.py](harness/loop.py) 系统提示:找物体一律先 find_objects;“让手离它 N 厘米”用 touch_object(gap_cm=N);reachable_by 有这只手就直接做,不要自己猜。测试 `test_touch_gap_one_cm`、`test_tool_texts_steer_the_model`。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real "please point to the pink object on the table in front of you by using your right hand and put your right hand just 1cm away from it"
```

## “看左边的人”转不了头、“抬头”被拒三次

trace `traces/20261113_002857.jsonl`、`traces/20261113_002928.jsonl`。

**1. 转头失败:`直线沿途 0% 处撞:<octomap>↔arm_r_end_link`**。0% 就是起点:上一条指令击掌后右手还挨着人手(人手在点云里),起点就“撞”。只动头,右手根本不动,但 `WholeBody.plan` 一看起点撞就什么都不让动,中转、RRT 也全被拒(15 s、2799 次碰撞检查)。

修:[whole_body.py](harness/motion/whole_body.py) `plan()` 先查起点的接触对;**只放过“起点就有 + 两边这次都不动”**的接触(`_moves`:按 link 名前缀判断,`arm_r_* / tcp_r_*` 属右臂、`head_link*` 属头,腰一动上面全动;`<octomap>` 和碰撞体不动)。新出现的接触、涉及要动的部件的接触照样拦。`_valid` 拆出 `_contacts`;JointDetour 用同一个 `_valid`,中转 / RRT 也一样放过。规划摘要里写明放过了哪些。

**2. “抬头”:`目标 -1.0° 超出限位 [-1.0°, 30.0°]`、`0.0° 超出限位 [-1.0°, 30.0°]`**。gateway 还要留 `LIMIT_MARGIN_RAD` 0.02 rad(1.15°),实际可用 0.15°~28.85°,报错却写 URDF 原始限位,模型按它试了 −1°、0° 都被拒,最后还说“已经抬到极限”(其实在 17.9°,还能抬 17.8°)。

修:[gateway `_plan_and_run_joints`](harness/gateway.py):目标超出可用范围就**停在可用范围边上**并在结果里写 `clamped_to_limit`(写的是真正可用的范围);已经在边上再往外转才报“已经在关节限位边上”。“抬头到底”一次就做完。

测试:`test_joint_urdf_limit_clamps_to_usable_range`(替换原来的“超限就拒”)、新 [tests/test_whole_body_contacts.py](tests/test_whole_body_contacts.py)(哪些 link 跟着动、起点已有的不相关接触放过、新接触照样拦)。原限位下 125 个全过;运控 + 接触 20 个全过。**真机没跑**。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real "看左边的人"
python3 scripts/run_task.py --real "抬头"
source /opt/ros/humble/setup.bash && python3 -m pytest tests/test_whole_body_contacts.py -q
```

## “和我击掌”没碰上去:不是约束、不是解算、不是深度,是缺“碰到物体”的工具

trace `traces/20261113_002016.jsonl`:`find_objects(hand)` → `look_at_object(hand_1)`(转头后手在画面中央,偏 2.5/−7.5 px)→ `point_at_object(hand_1, right)`(指向误差 1.6°)→ 模型停下问“要不要真的迎上去碰一下,还差 34cm”。没有任何规划失败。

逐个排除:

- yaml 约束:没有任何工具返回被拒。
- 解算:根本没发起“碰上去”的规划。
- 深度 / TF:分割日志 `traces/sam3/20261113_002018_hand/` 里 hand_1 掩码 3005 像素、99% 有深度,深度 0.58~0.69 m 单峰,没混进背景;转头后重新识别(`..._hand_after`)位置只差约 2 cm,说明深度和坐标链都对。手离右肩 0.68 m,够得着。

真正原因:没有“把末端送到物体上”的工具——`point_at_object` 只转方向,`approach_object` 只能停在上方 18~40 cm;要碰上去模型得自己算偏移(还得扣掉 12.8 cm 的夹爪长),而且对象是人手,它就停下来问了。

改动:

- 新工具 `touch_object(object_id, arm, gap_cm)`([gateway `_touch_object`](harness/gateway.py)):夹爪朝物体,末端停在物体前“夹爪长 0.128 m + gap(默认 2 cm)”;规划自动走接触兜底;结果报 `tip_to_object_m`。配置 `motion.touch`。
- [系统提示](harness/loop.py):击掌 / 碰拳 / 碰一下用 touch_object,用户已经要求接触,直接做不用再确认。
- 离线复现(日志里的右臂关节角、hand_1 坐标、真实 IK、手周围 16 cm 当障碍):先避障到目标前 3 cm,再不查碰撞直线过去;夹爪尖离手 2.2 cm,方向误差 0°。测试 `test_touch_hand_stops_tool_length_in_front`、`test_agent_high_five_without_asking`;原限位下 121 个全过。**真机没跑**。

命令:

```bash
# 终端 1
cd ~/sam3_service && .venv/bin/python server.py
# 终端 2
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real "和我击掌"
python3 scripts/run_task.py --real "你是我的拳击陪练,出右拳和我的右拳对在一起"
```

## “抓取最近的粉色积木”为什么失败、怎么成功

trace `traces/20261113_001546.jsonl`:`find_objects(pink block)` 找到 (0.749, −0.177, 0.736),`reachable_by: [right]`;之后 `approach_object(right, tool_axis=down)` 不同高度连续失败(IK 差 14~24mm),模型手动挪了几次也不行,最后建议“腰向右转”。

离线分析(日志里的腰 [−39.85, 89.84, −49.69, 0, 0] + 右臂关节角,真实 URDF):

| 情况 | 积木上方 25 / 18 / 15 cm 夹爪竖直朝下 |
|---|---|
| 现在的腰 | 都不行:夹爪朝下时腕心必须在积木正上方,腕心离肩 0.59 m,L1+L2 只有 0.61 m,胳膊几乎伸直 + 腕关节限位 |
| 腰向右转 15° / 30°(模型的建议) | 都不行,**更远**(右肩往后走,腕心离肩 0.65 / 0.71 m) |
| 上身前倾 10°(body j3 +10°) | 都能 |
| 腰不动,夹爪离竖直 10° | 都能(夹爪往外斜一点,腕心就往肩收回几厘米) |

结论:积木在右臂“夹爪竖直朝下”的极限边上;`reachable_by` 按物体中心离肩 0.72 m 判,对“从正上方抓”偏乐观(上方抓要求腕心水平离肩 ≲ 0.58 m)。

改动:

- [`RosBackend.plan_arm_to`](harness/backends/ros.py):`tool_axis` 严格做不到时,**同一位置**找最接近的朝向(`tilt_search` 后撤 0),离目标方向 ≤ `motion.tool_axis_tolerance_deg`(15°)就接受;结果带 `tool_axis_miss_deg`。之后才是后撤 / 接触兜底。
- 接触兜底预检查:要求了 `tool_axis` 就按“容差内最接近这个朝向”查 IK,不再只看位置(这次每次白试 8 个接近点);够不着时提示“可以让上身前倾”。
- 替身手臂 + 真实 IK 复现:两种起始右臂、上方 25 / 18 cm,现在都能规划,夹爪离竖直 5~6°。测试 `test_approach_reports_tool_axis_miss`;原限位下全过。**真机没跑**。

还缺(未做):① 够不着时自动用腰(前倾 j3)帮忙,不靠模型猜——模型猜的方向是反的;② “抓取”技能:上方预抓位 → 开夹爪 → 下降(接触兜底)→ 合夹爪 → 抬起,现在模型只能一步步拼。

命令:

```bash
# 终端 1
cd ~/sam3_service && .venv/bin/python server.py
# 终端 2
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real "右臂移到最近的粉色积木上方,夹爪朝下"
# 想严格竖直:先前倾上身再试
python3 scripts/run_task.py --real "上身前倾10度,然后右臂移到粉色积木上方,夹爪竖直朝下"
```

## “拳击陪练”一段的 bug 分析

trace `traces/20261113_000405.jsonl`:“你是我的拳击陪练,出右拳和我的右拳对在一起”,用满 8 轮后停止,没完成。

| # | 现象 | 原因 | 修 |
|---|---|---|---|
| 0 | “超过最大步数 8” | `agent.max_steps` 是一条指令最多问模型几轮,不是记忆 | 改 20。另:**每条指令都是全新对话,模型不记得上一条指令** |
| 1 | 接触兜底先动了一段,然后“最后一段直线算不出来:差 286mm”,两次 | 目标 x=1.10 是**够不着**,不是被点云挡住;兜底没区分,而且最后一段等到执行时才算 | [`_plan_contact`](harness/backends/ros.py):先查“不考虑碰撞能不能解 IK”,解不出直接报“超出手臂范围”;每个接近点规划时就用积分笛卡尔验算最后一段,走不到换下一个 |
| 2 | `offset [25,0,0] + tool_axis forward` 报 ok,实际只动 3.5cm | 后撤搜索从目标往回撤了 25cm,把偏移抵消了,`target_xyz_m` 被悄悄换掉 | [gateway](harness/gateway.py) `move_arm_end`:给了 `offset_cm` 就不后撤;只给朝向才后撤 |
| 3 | `turn_deg: 0.0`,实际转了 50° | 这个字段只算 `rotate_deg` | 改名 `requested_rotate_deg`,执行后加 `actual_turn_deg`(起止姿态实际差多少度) |
| 4 | 模型选了 `fist_2`(离右肩 1.07m) | `find_objects` 不说哪只手够得着 | 每个物体加 `shoulder_distance_m`、`reachable_by`(离肩 ≤ `motion.arm_reach_m` 0.72m);系统提示“要碰它先看 reachable_by”。真机那次:fist_1 离右肩 0.712m(伸直极限 0.70m,表面更近),fist_2 1.068m |
| 5 | 模型拿 `move_arm_end` 硬凑偏移去碰拳头 | **缺能力**:`approach_object` 只能停在物体上方 18~40cm,没有“末端送到物体上接触”的工具 | 未做,待定 |

验证:替身手臂 + 真实 IK(日志里的腰、右臂关节角):够不着的目标 → 报“超出范围”、机器人 0 次运动;够得着但被挡的点 → 选 8cm 接近点,最后一段不查碰撞到达(差 1mm)。测试 `test_retreat_only_when_no_offset`、`test_find_objects_reports_reach`;原限位下 118 个全过,现配置仍是那 3 个限位测试失败。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real "你是我的拳击陪练,出右拳和我的右拳对在一起"
python3 -m pytest tests -q
```

## 规划失败的接触兜底:先避障到附近,最后一段不查外界碰撞

问题:头部点云把所有东西(桌上的物品、人的手)都扫成碰撞体,机器人永远不可能主动接触任何物品——目标在障碍里,规划必失败。
用户要求:规划失败时先别告诉模型,先到过去路上离目标最近的可达位置,再不检测外界碰撞笛卡尔移动过去。设计见 [docs/goal_pose.md#规划失败时的接触兜底](docs/goal_pose.md)。

- [`RosBackend._plan_contact`](harness/backends/ros.py):所有正常规划(含后撤)都失败后,沿“目标→当前末端”试 3、5、8…30 cm 的接近点,每个点完整正常规划,第一个成功的用;返回 `kind: "contact"` 的两段式计划。
- [`RosBackend._execute_contact`](harness/backends/ros.py):先执行避障段;到了从实际位置、实际姿态 `plan_cartesian(check_collision=False)` 直线到目标,0.03 m/s,超过 0.30 m 就停。
- [arm_motion.py](harness/motion/arm_motion.py) `plan_cartesian` 加 `check_collision`(robot_harness 自己加的)。
- [gateway](harness/gateway.py) `_contact_info`:approach_object / move_arm_end / point_at_object 结果里带 `precontact_xyz_m`、`unchecked_m` 和 note,模型如实转告。
- [configs/harness.yaml](configs/harness.yaml) `motion.contact_fallback`:enabled、backoffs_cm、max_unchecked_m、speed_m_s。
- 注意:MoveIt 自碰撞和外界碰撞一起查,最后一段**自碰撞也没查**(短、直线、从已验证的姿态出发)。
- 验证:替身手臂(目标周围 6 cm 是障碍)→ 选 8 cm 接近点,最后一段不查碰撞到达目标。假机器人测试 `test_contact_fallback_is_reported_not_failed`;原限位下 116 个全过。**真机没跑**。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real --plan-only "和你的右夹爪和我的右手击掌"
python3 scripts/run_task.py --real "和你的右夹爪和我的右手击掌"
# 不想要接触兜底:configs/harness.yaml 里 motion.contact_fallback.enabled: false
```

# 2026.11.12

## 识别升级:调研

问题:[`locate_object`](harness/gateway.py#L69) 只能按颜色找,且 gateway 只有一个目标槽位 [`self.target`](harness/gateway.py#L46),“先看 A 再看 B 再回 A”做不到。

- 物体怎么传给运动工具:识别返回编号、运动只收编号,坐标留在 gateway → [docs/object_handles.md](docs/object_handles.md)(**未实现**)。参考了 [Code as Policies](https://arxiv.org/abs/2209.07753) 第 III 节和 [SayPlan](https://arxiv.org/abs/2307.06135v2)。
- 识别器选型:SAM 3 首选,Grounded-SAM-2 备选,YOLO-World 不需要 → [docs/perception_plan.md §1](docs/perception_plan.md#1-sam-3文字--掩码)。
- 分析了 `~/g1_pointcloud_grasping` 的点云聚类(RANSAC 桌面 + DBSCAN + 贴桌过滤 + 几何):无语义但直接给 3D 几何,和 SAM 3 互补 → [docs/perception_plan.md §2](docs/perception_plan.md#2-g1-点云聚类无标签)。建议顺序:先聚类、再 SAM 3 贴标签、顺带把桌面加进 planning scene([§3](docs/perception_plan.md#3-对比与建议))。

## SAM 3 独立服务

SAM 3 要 Python ≥ 3.12,ROS Humble 是 3.10,所以在 `~/sam3_service` 单独起一个本机 HTTP 服务 → [docs/sam3_service.md](docs/sam3_service.md)。

- 环境用 uv 建(比 conda 快,`.venv` 就在目录里):Python 3.12.13、torch 2.10.0+cu128、SAM 3 提交 `f92a5ef`。安装命令见 [安装](docs/sam3_service.md#安装从零重建)。
- 写了 [`server.py`](../sam3_service/server.py)(模型加载一次,一张图多个短语只算一次图像特征)、`sam3_client.py`、`test_client.py`、`capture_head.py`(后来挪进本仓库,见下一节)。
- 机器人关机前用 [`capture_head.py`](scripts/capture_head.py) 录了 5 帧(彩色 + 深度 + 头部位姿),现在在 `recordings/head_20261112_015829/`,离线也能测“掩码 + 深度 → 3D”。画面里有人,只留本机。

问题记录(详细见 [踩坑记录](docs/sam3_service.md#踩坑记录)):

| 问题 | 原因 → 解决 |
|---|---|
| `hf auth login` 报 `Unknown scheme for proxy URL 'socks://...'` | 终端代理写成 `socks://`,`hf` 不认 → 命令前设 `all_proxy=http://127.0.0.1:7897` |
| `Invalid user token` | 终端不回显,粘贴出错 → 改用浏览器登录 |
| `pkg_resources` 报 `NoneType` | 克隆目录叫 `sam3/`,`import sam3` 找到的是文件夹不是包 → 改名 `sam3_repo/` 重装 |
| 缺 `einops` 等 | 官方列为可选 → 手动装 `einops pycocotools psutil scipy` |
| 权重下好后,终端直接 `server.py` 仍报 `socks://` | 启动时 SAM 3 每次都联网查版本,`HF_HUB_OFFLINE` 无效 → [`load_model`](../sam3_service/server.py) 先查本机缓存,传 `checkpoint_path`,启动不联网 |
| 权重下到 645 MB 会话重启后从 0 开始,随后卡死 | Xet 走代理不稳、不能续传 → `HF_HUB_DISABLE_XET=1 hf download`,`setsid` 后台跑 |

命令(安装见 [安装](docs/sam3_service.md#安装从零重建)):

```bash
# 登录 Hugging Face(一次)。终端代理是 socks://,hf 不认,临时改 http://;选浏览器登录
cd ~/sam3_service
all_proxy=http://127.0.0.1:7897 ALL_PROXY=http://127.0.0.1:7897 .venv/bin/hf auth login

# 下载权重(一次,3.4 GB)。关掉 Xet 走普通 HTTP,断了重跑同一条会续传
HF_HUB_DISABLE_XET=1 https_proxy=http://127.0.0.1:7897 .venv/bin/hf download facebook/sam3 sam3.pt config.json

# 启动服务(权重在本机就不联网,约 7 s;看到“服务已启动 http://127.0.0.1:8765”)
cd ~/sam3_service && .venv/bin/python server.py

# 查服务状态、显存
curl -s localhost:8765/health

# 关掉服务(释放约 4 GB 显存)
pkill -f "\.venv/bin/python server.py"

# 单张图测试(不需要 ROS),结果图 → traces/sam3_color_04.jpg
cd ~/robot_harness
python3 scripts/sam3_test_image.py recordings/head_20261112_015829/color_04.png "red block" "pen"

# 录 5 帧头部彩色 + 深度 + 头部位姿 → recordings/head_<时间>/
source ~/agi@g2_robot/env_g2.sh
python3 scripts/capture_head.py
```

实测(详见 [测试结果](docs/sam3_service.md#测试结果)):9 个短语在录的帧上基本都对,70 像素的蓝积木也能分出;一张图 1 个短语约 320 ms,每多一个 +45 ms;显存常驻 3.4 GB、峰值 5.1 GB。问题:红积木也被 `orange block` 匹配(0.66),接入时要做跨短语掩码去重;`box` 这类泛称不稳。

待办:

- [x] 权重下完后用录的帧测分数、耗时、显存。
- [ ] 真机跑 [`sam3_live.py`](scripts/sam3_live.py)。
- [ ] 真机 `--plan-only` 再真动:“看向最近的手机”([look_at_object](docs/look_at_object.md))。
- [ ] 把 SAM 3 接进 harness:[`Sam3Client`](harness/perception/sam3_client.py#L25) → 掩码 + 深度 → [`points_base`](harness/perception/scene3d.py#L36),跨短语去重。
- [ ] 物体编号表([设计](docs/object_handles.md#要改的地方))。
- [ ] 搬 G1 聚类 → `list_objects()`。
- [ ] 桌面作为碰撞体加进 planning scene。
- [ ] 真机 `--plan-only` 验证 11.11 的新运动工具。

## SAM 3 实时画面 + 代码重新分家

需求:接 G2 头部 RGB,终端里随时改物体名,画面上实时盖掩码。典型的快输入(相机 27 fps)慢处理(SAM 3 约 0.35 s/帧)。

**代码放哪**:按 Python 环境分界。`~/sam3_service` 只留 [`server.py`](../sam3_service/server.py)(3.12,只管“图 + 短语 → 掩码”,不知道 ROS);客户端、实时画面、录相机这些要用 ROS 或要被 harness import 的,全部挪进本仓库。不放 `agi@g2_robot`。理由见 [docs/sam3_service.md#代码放在哪](docs/sam3_service.md#代码放在哪按-python-环境拆成两边)。

| 原位置 | 现在 |
|---|---|
| `~/sam3_service/sam3_client.py` | [harness/perception/sam3_client.py](harness/perception/sam3_client.py),新增 [`segment_jpeg`](harness/perception/sam3_client.py#L54) |
| `~/sam3_service/test_client.py` | [scripts/sam3_test_image.py](scripts/sam3_test_image.py),结果图 → `traces/sam3_*.jpg`(gitignore) |
| `~/sam3_service/capture_head.py` | [scripts/capture_head.py](scripts/capture_head.py),输出 → `recordings/head_<时间>/`(gitignore) |
| `~/sam3_service/captures/20261112_015829/` | `recordings/head_20261112_015829/`(画面里有人,不进 git) |
| — | 新 [scripts/sam3_live.py](scripts/sam3_live.py) |

**[`sam3_live.py`](scripts/sam3_live.py) 的线程设计**(详见 [快输入、慢处理](docs/sam3_service.md#快输入慢处理)):

- 4 个线程:ROS 回调(`CameraSubscriber.on_image`)、分割(`segment_loop`)、输入(`input_loop`)、主线程显示。
- 相机和分割之间不用队列,用 `LatestSlot`:覆盖写、`threading.Condition` 等新帧,中间帧直接丢;ROS 订阅也设 `KEEP_LAST depth=1`。
- 零拷贝:格子里只存 ROS 消息引用;`msg.data`(`array.array`,本身就是 JPEG)用 `memoryview` 原样发给服务。为此 [`server.py`](../sam3_service/server.py) 的 `/segment` 新增 `Content-Type: image/jpeg` 写法,不再 base64。显示线程有新帧才 `np.frombuffer` + 解码。
- 改短语时 `Prompts.version + 1`,旧短语的结果不画。

命令:

```bash
# 终端 1:模型服务
cd ~/sam3_service && .venv/bin/python server.py

# 终端 2:实时画面。之后在终端 2 输入 "green block, black bag" 回车换物体;空行清空;q 退出
source ~/agi@g2_robot/env_g2.sh
cd ~/robot_harness && python3 scripts/sam3_live.py "red block" "pen"

# 掩码和画面严格对齐(画面慢约 0.35 s)
python3 scripts/sam3_live.py --sync "pen"
```

验证:没有真机,写了个假相机在 **ROS domain 77** 以 30 Hz 回放录的帧(没用 domain 4),把 `cv2.imshow` 换成存图跑无窗口测试:画面 27 fps、分割 2~3 fps、掩码延迟 350~410 ms、15 s 跳过 288 帧旧图无堆积;中途改短语(含中文逗号)、清空都即时生效。**真机未测。**

## 技能:看向最近的手机(SAM 3 + 头部规划组)

需求:说“看向最近的手机”,模型用 SAM 3 工具传 `phone`,机器人动头部规划组让手机在画面中央;分割画面和掩码存日志。设计和流程图见 [docs/look_at_object.md](docs/look_at_object.md)。

| 新 / 改 | 内容 |
|---|---|
| [harness/tools/objects.py](harness/tools/objects.py#L32) | 两个新工具:`find_objects(query)`(不动)、`look_at_object(object_id)`(只动头) |
| [`Gateway._find_objects`](harness/gateway.py#L80) | 只收英文短语;结果按编号存进 [`self.objects`](harness/gateway.py#L48),同一短语重新找时旧编号作废 |
| [`Gateway._look_at_object`](harness/gateway.py#L102) | 编号 → 头部角度 → 走 `move_joints` 同一条 [`_plan_and_run_joints`](harness/gateway.py)(限位、碰撞、只规划、人按 y)→ [`_check_centered`](harness/gateway.py#L138) 再分割一次量偏差 |
| [`RosBackend.find_objects`](harness/backends/ros.py#L147) | 一帧 RGBD → [`Sam3Client.segment`](harness/perception/sam3_client.py#L47) → [`objects_from_masks`](harness/perception/sam3_objects.py#L29) → [`save_segmentation`](harness/perception/sam3_objects.py#L70) |
| [`RosBackend.head_angles_to`](harness/backends/ros.py#L176) → [`HeadAim.solve`](harness/perception/head_aim.py#L51) | URDF 链 `base_link→head_link3`(5 腰 + 3 头)+ 相机外参;只改 yaw、pitch,高斯-牛顿,夹在限位里 |
| [harness/backends/fake.py](harness/backends/fake.py#L42) | 假 `find_objects` / `head_angles_to`,不用 ROS 和 SAM 3 也能测 |
| [configs/harness.yaml](configs/harness.yaml) | `vision.sam3`(url、阈值、超时、最少深度像素)、`motion.look_at_match_m` |
| [harness/loop.py](harness/loop.py) | 系统提示加一句:说物体名用 `find_objects`,“最近的”选第一个 |
| `.gitignore` | `traces/sam3/`(画面里可能有人) |

- 日志:每次分割一个文件夹 `traces/sam3/<时间>_<短语>[_after]/`,里面 `color.jpg`、`overlay.jpg`、`mask_<编号>.png`、`depth.png`、`result.json`。转头前后各一个,可以对比。
- 头部几何(从 URDF 算):零位时相机在 base_link (0.20, 0.03, 1.68),光轴朝下约 25°;yaw 限 ±44.7°,pitch 限 −1°~30°。桌上离得很近的东西 pitch 不够,会停在限位并返回 `head_at_limit` 和剩余偏差。
- 验证:`tests/test_sam3_look.py` 18 个,全部 82 个通过。录的真机帧离线跑通感知链:`phone` 找到人手上的手机(1.75 m,0.82 分),3D 点反投影和掩码中心差 < 1 px。**真机转头未测**。

命令:

```bash
# 终端 1:SAM 3 服务
cd ~/sam3_service && .venv/bin/python server.py

# 终端 2:先只规划(不动),看算出来的头部角度和日志
source ~/agi@g2_robot/env_g2.sh
cd ~/robot_harness
python3 scripts/run_task.py --real --plan-only "看向最近的手机"

# 确认没问题再真动(会问 y)
python3 scripts/run_task.py --real "看向最近的手机"

# 看最新一次分割日志
ls -t traces/sam3 | head
xdg-open traces/sam3/$(ls -t traces/sam3 | head -1)/overlay.jpg

# 测试
python3 -m pytest tests/test_sam3_look.py -q
```

## 问题:“右臂末端放在最近的黄色物品上空 30 厘米”规划失败

现象(trace `traces/20261112_222155.jsonl`):`locate_object(yellow)` 有 2 个候选,返回 `"selected": "离左臂末端最近"` 的那个(y=+0.347,在机器人左侧,离左手 12 cm);随后 `approach_object(arm=right)` 让右臂去够它,移动 1.06 m,直线差 210 mm、绕障 IK 无解。

原因:

1. [`locate_object`](harness/backends/ros.py) 的“最近”写死按**左臂**末端算([`merge_frames`](harness/backends/color_object.py#L40) 的 reference 是 `left_end_xyz()`),不知道接下来要用右臂。用户说的“最近”应该按右臂算,很可能是另一个黄色物体。
2. 这次 gateway 没拦住,是因为 [configs/harness.yaml](configs/harness.yaml) 被放宽了:`max_travel_m` 0.70 → 1.70,右臂 `workspace.y` 上限 0.20 → 1.20(左臂 x、y 也放大了)。按原来的值,y=0.347 和 1.06 m 的移动都会在规划前被拒。规划器最终正确地拒绝了,机器人没动。

改动:

- `locate_object` 加 `arm` 参数:同色多个时选离**这条手臂**末端最近的;结果 `selected` 写“离右臂末端最近”,字段 `distance_to_left_end_m` 改名 `distance_to_arm_end_m`。改了 [gateway `_locate`](harness/gateway.py)、[ros.py](harness/backends/ros.py)、[fake.py](harness/backends/fake.py)、[工具说明](harness/tools/objects.py)、[系统提示](harness/loop.py)(“两次的 arm 要一致,最近指离这条手臂最近”)。
- [`_approach`](harness/gateway.py):定位时用的手臂和现在要动的手臂不一致就拒绝,让模型先 `locate_object(arm=right)`。
- 新测试 `test_locate_uses_chosen_arm_as_reference`、`test_approach_with_other_arm_refused`。

测试:用原来的限位 84 个全过;用现在放宽的 `harness.yaml`,`test_workspace_refused`、`test_long_travel_refused`、`test_arm_target_outside_workspace_refused` 3 个失败(它们就是测这些限位的)。**配置没动,放宽是否保留由人决定**;保留的话这 3 个测试要跟着改。

命令:

```bash
cd ~/robot_harness
python3 -m pytest tests -q
# 重跑这条指令,先只规划
python3 scripts/run_task.py --real --plan-only "右臂末端放在最近的黄色物品上空30厘米处"
# 看哪个黄色物体被选中(绿框)
xdg-open traces/last_object.jpg
```

## 问题:find_objects 找到了 phone_1,approach_object 却说“还没有定位物体”

现象(trace `traces/20261112_222924.jsonl`):“右臂末端抵达最近的手机上空30厘米处” → `find_objects(phone)` 成功(phone_1,`xyz_m` [0.755, -0.148, 0.708])→ `approach_object(arm=right, standoff_cm=30)` 返回“还没有成功定位物体,先调用 locate_object”,模型转头问用户手机是什么颜色。

原因:`approach_object` 只认 `locate_object`(按颜色)存的单槽位 `self.target`;`find_objects` 的结果存在编号表 `self.objects`,两边没打通([object_handles.md](docs/object_handles.md) 里写的“approach_object 还在用旧单槽位”)。

改动:

- [`_approach`](harness/gateway.py) 加 `object_id`:给了就从编号表取坐标(查编号存在、没过期);不给就照旧用颜色定位的结果(含上一节的“定位和动的必须是同一条手臂”)。目标点、防穿桌高度、到物体距离都改用这个坐标。
- [工具说明](harness/tools/objects.py):`approach_object` 加 `object_id`,写明“来自 find_objects 时填”。[系统提示](harness/loop.py):手臂去物体上方用 `approach_object(object_id=…)`。
- 新测试:右臂到 phone_1 上方 30 cm、编号不存在、整条模型对话“右臂末端抵达最近的手机上空30厘米处”。

测试:原来的限位下 87 个全过;现在放宽的 `harness.yaml` 下仍是那 3 个限位测试失败(同上一节)。

命令:

```bash
# 终端 1:SAM 3 服务
cd ~/sam3_service && .venv/bin/python server.py
# 终端 2:先只规划
source ~/agi@g2_robot/env_g2.sh
cd ~/robot_harness
python3 scripts/run_task.py --real --plan-only "右臂末端抵达最近的手机上空30厘米处"
python3 -m pytest tests/test_sam3_look.py -q
```

## 资产整理到 ~/ai_assistant + 能力盘点

暂停改代码。把 harness 的成果按 `~/ai_assistant` 的规矩(原样复制、不改 import、每个目录一份 README、根 README 登记“任务路由”)放进去:

| 新目录 | 内容 |
|---|---|
| `~/ai_assistant/src/g2_llm_harness/` | loop、llm、gateway、tools、backends、`harness.yaml`、`run_task.py`、3 个测试文件 |
| `~/ai_assistant/src/g2_object_perception/` | RGBD 取帧/3D、HSV 颜色、SAM 3 客户端/服务/掩码→3D、HeadAim、实时分割、录帧、取色 |
| `~/ai_assistant/src/g2_whole_body/` | WholeBody、JointDetour、joints、poses.yaml、goto_pose |
| `~/ai_assistant/src/g2_llm_harness/TOOLS_MAP.md` | 每种能力的候选工具、现在最好的、缺什么 |

触发:真机“右臂末端抵达最近的手机上空30厘米处”成功(直线 16.7 cm,误差 0 mm),但末端是**保持当前姿态的纯平移**([`plan_arm_to`](harness/backends/ros.py) 传 `rpy=None`,`full` 模式锁 6 维),手机再往中线靠就够不到。
`ArmSolver` 已有 `axis`(只对准工具轴)/ `pos`(只管位置)模式,`plan_cartesian` 也收 `mode`,只是 harness 没用;`plan_detour` 终点还只支持 `full`。

缺口(按优先级,详见 TOOLS_MAP.md):末端姿态放松(axis 朝下 → pos → 用腰)> 桌面进碰撞检查 > 物体几何(主轴/顶面/尺寸)> 吸盘工具 > 跨短语去重 + 多帧 > 评测集 > 颜色定位也进编号表。

命令:

```bash
# 看整理结果
cat ~/ai_assistant/src/g2_llm_harness/TOOLS_MAP.md
git -C ~/ai_assistant status --short
```

## 录当前右臂 + 左臂摆成对称

新脚本 [scripts/record_pose.py](scripts/record_pose.py):读 `/joint_states` 录进 [configs/poses.yaml](configs/poses.yaml)(机器人不动);`--mirror-left` 再生成 `<名字>_mirror`,只含左臂。
然后用已有的 [scripts/goto_pose.py](scripts/goto_pose.py) 去这个位姿:沿途整机碰撞检查、会撞就绕、打印关节变化、按 y 才动。

- 对称换算:左臂关节 i = (−, +, −, +, −, +, −)[i] × 右臂关节 i。用 G2 URDF 正运动学验证,左末端位姿 = 右末端关于躯干 xz 面的镜像,误差 1e-5 m;左右臂限位镜像后完全一致。
- 对称是相对躯干的:腰有侧倾(body 4)或转腰(body 5)时,相对 base_link 就不是对称的。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
cd ~/robot_harness
# 录当前右臂为 right_now,同时生成左臂对称位姿 right_now_mirror(不动)
python3 scripts/record_pose.py right_now --groups right_arm --mirror-left
# 左臂去对称位姿(打印关节变化,按 y 才动)
python3 scripts/goto_pose.py right_now_mirror
# 同名已存在时覆盖
python3 scripts/record_pose.py right_now --groups right_arm --mirror-left --force
```

## “抵达初始位姿”:把当前全身姿态录成 init

需求:对 harness 说“抵达初始位姿”,腰、头、双臂回到现在这个样子。

- [scripts/record_pose.py](scripts/record_pose.py) 加 `--note`:位姿含义写进 `poses.yaml` 的 `note`。
- [harness/tools/motion.py](harness/tools/motion.py) `make_motion_tools(pose_names, pose_notes)`:`go_to_pose` 的说明里列出“init = 初始位姿;…”,模型听到中文说法能对上位姿名。[run_task.py](scripts/run_task.py) 把每个位姿的 note 传进去(每条指令重读 poses.yaml,录完不用重启)。
- 走的是已有的 `go_to_pose` → [`_plan_and_run_joints`](harness/gateway.py):URDF 限位 → 整机碰撞检查,会撞就经其他位姿中转 / 关节 RRT → 人按 y。不受 `max_joint_delta_deg` 限制(位姿是人录的);4 组一起动时速度取最慢的腰 0.10 rad/s。
- 新测试 `test_pose_notes_reach_the_model`、`test_agent_go_to_initial_pose`。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
cd ~/robot_harness
# 1. 把现在的全身姿态录成 init(机器人不动;已存在就用 --force 覆盖)
python3 scripts/record_pose.py init --note 初始位姿 --force
# 2. 以后跟 harness 说(先只规划,再真动)
python3 scripts/run_task.py --real --plan-only "抵达初始位姿"
python3 scripts/run_task.py --real "抵达初始位姿"
# 不经过大模型,直接去
python3 scripts/goto_pose.py init
```

## 夹爪:打开 / 闭合 / 等几秒

需求:“打开/闭合右手夹爪”“你先打开夹爪过5秒再闭合”;按两只手都装了来设计,没装的手失败时 harness 要说清楚。详见 [docs/gripper.md](docs/gripper.md)。

查找过程:`ros2 topic list`(domain 4)没有夹爪话题;`/gdk/joint_state` 不含夹爪,硬件插件也不管(`arm_control_ros.cpp` 注释);最后在 GDK(`~/.cache/agibot/app/gdk`)里找到 `Robot.move_ee_pos` / `get_end_state` / `get_whole_body_status`,以及伺服话题里的夹爪关节名 `idx71_gripper_r_inner_joint1`。
`get_whole_body_status` 实测:右手 `omnipicker`、错误码 0;左手型号为空、错误码 2147549184。`get_end_state` 一直 `No end state message found`,所以夹爪位置读不到。

| 新 / 改 | 内容 |
|---|---|
| [harness/backends/gripper_gdk.py](harness/backends/gripper_gdk.py) | `GdkGripper`:状态 + 开合;默认走伺服话题 `/gdk/joint_control`(手臂也在伺服,GDK 文档要求末端同走伺服),备选 `move_ee_pos` |
| [harness/gateway.py](harness/gateway.py) | `gripper`(算一次运动、只规划不动、按 y)、`wait`(≤ `max_wait_s`);`get_robot_state` 带夹爪状态 |
| [harness/tools/motion.py](harness/tools/motion.py)、[harness/loop.py](harness/loop.py) | 工具说明;系统提示“过几秒再…在两步之间调用 wait” |
| [harness/backends/ros.py](harness/backends/ros.py)、[fake.py](harness/backends/fake.py)、[run_task.py](scripts/run_task.py) | `gripper_state` / `gripper_command`;假夹爪可设某只手没装 |
| [scripts/gripper_cli.py](scripts/gripper_cli.py) | 不经过大模型单独测 |
| [tests/test_gripper.py](tests/test_gripper.py) | 10 个:开合、左手没装报失败、参数错、只规划、用户拒绝、wait 上下限、状态、整条“开→等 5 秒→合”、模型能看到左手失败原因 |

验证:`gripper_cli.py status` 在真机上只读跑通(结果同上)。**真机开合还没动过**。测试:原限位下 99 个全过;现在放宽的 harness.yaml 下仍是那 3 个限位测试失败。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh          # 夹爪走 GDK,必须
cd ~/robot_harness

# 1. 只读:两只手的夹爪型号、错误码
python3 scripts/gripper_cli.py status
# 2. 单独测右手(按 y 才动;手臂别动,旁边有人)
python3 scripts/gripper_cli.py open --arm right
python3 scripts/gripper_cli.py close --arm right
# servo 不行再试另一种
python3 scripts/gripper_cli.py open --arm right --interface move_ee_pos

# 3. 通过 harness
python3 scripts/run_task.py --real "打开右手夹爪"
python3 scripts/run_task.py --real "你先打开右手夹爪过5秒再闭合"
python3 scripts/run_task.py --real "打开左手夹爪"          # 应该报“左手没有识别到夹爪”

python3 -m pytest tests/test_gripper.py -q
```

## 夹爪真机:右手张开成功

`python3 scripts/run_task.py --real` → “张开右夹爪” → `gripper(right, open)` → 按 y → 夹爪张开(伺服方式 `/gdk/joint_control`)。结果 `ok: true, verified: false`(读不到位置)。
终端里两行 `E1112 ... hal_impl.cpp:305] [GDK]No end state message found` 不是 harness 出错:是 GDK C++ 库在 `get_end_state()` 读不到时自己打的日志,动作前后各读一次所以两行。
改动:[`GdkGripper._position`](harness/backends/gripper_gdk.py) 读不到一次就记下,本进程不再读(最多一行)。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real "你先打开右手夹爪过5秒再闭合"
```

## 取消逐次按 y,改为自动执行

用户决定:点云 octomap 避障 + 关节速度慢,不再每次运动都按 y。改成一个总开关,默认自动执行,随时能恢复。

- [configs/harness.yaml](configs/harness.yaml) `motion.confirm_motion: false`(去掉了单独的 `gripper.confirm`)。
- [run_task.py](scripts/run_task.py):真机默认 `announce`(打印 `[自动执行] <计划>` 后直接执行);`confirm_motion: true` 或 `--confirm` 时 `ask_user`(按 y)。假机器人不问也不打印。
- [gateway.py](harness/gateway.py):手臂、关节、夹爪都只走 `self.confirm(要做什么)`,提示文字里不再写“输入 y”。
- [goto_pose.py](scripts/goto_pose.py)、[gripper_cli.py](scripts/gripper_cli.py)、[motion_cli.py](scripts/motion_cli.py):默认直接执行,`--confirm` 才问 y。
- 没变的:规划/碰撞检查/限位/工作范围/只规划模式/每条指令运动次数上限;`goto_pose` 执行中 Ctrl-C 取消。
- 注意:避障靠 octomap,要用 `use_head_cloud:=true` 启动 `hardware.launch.py`;`false` 时 MoveIt 里没有桌子。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real                    # 自动执行
python3 scripts/run_task.py --real --confirm          # 临时恢复每次按 y
python3 scripts/goto_pose.py init                     # 直接去
python3 scripts/goto_pose.py init --confirm           # 先问 y
python3 scripts/gripper_cli.py open --arm right       # 直接开
```

## 手臂终点姿态:放松开关 + 三角函数舒适姿态

问题:末端“只会平移”(终点姿态锁成当前姿态),手机往中线靠就够不着;`via_then_rrt` 终点 IK 无解直接放弃。设计和推导见 [docs/goal_pose.md](docs/goal_pose.md)。

- 新 [harness/motion/goal_pose.py](harness/motion/goal_pose.py):`free_goal`(只管位置,先积分再多种子 IK)、`ComfortPose`(你在 genie 推的“肩-肘-腕-目标共面、临界值取中值、广义水平基面、肘外展”,改成够得着的版本 + 零空间拉肘 + 关节舒适打分)。
- [`RosBackend.plan_arm_to`](harness/backends/ros.py) 加 `orientation`:按顺序试 keep / comfort / free,第一个规划成功的就用;每种都还是“直线 → via_then_rrt”。规划器本身没改。
- [configs/harness.yaml](configs/harness.yaml) `motion.goal_orientation`:`approach_object: [comfort, keep, free]`,`move_arm_end: [keep, free]`;模型给了 `rotate_deg` 就只按它。
- 发现:G2 末端工具轴是 **Z**(腕心→末端);`ArmSolver` 的 `axis` 模式对准的是 X(sanofi 约定),所以“放松”用 `pos` 模式。
- 离线对比(40 个随机点):keep 22 能到 / 舒适分 1.56;free 24 / 1.37;comfort 18 / **0.87**,两者都到的 18 个里 comfort 更舒服 14 个。`find` 腰姿态下右臂去 (0.55, 0.15, 0.95):keep 无解,comfort / free 有解。
- 测试:新 [tests/test_goal_pose.py](tests/test_goal_pose.py)(几何自洽、能到位、不比放松别扭)+ 姿态顺序 1 个;原限位下 104 个全过。**真机没跑过**。

命令:

```bash
cd ~/robot_harness
source /opt/ros/humble/setup.bash
python3 -m pytest tests/test_goal_pose.py -q
# 真机:手机往中线放一点再试(先只规划,看摘要里“终点姿态 舒适/保持/放松”)
source ~/agi@g2_robot/env_g2.sh
python3 scripts/run_task.py --real --plan-only "右臂末端抵达最近的手机上空30厘米处"
python3 scripts/run_task.py --real "右臂末端抵达最近的手机上空30厘米处"
```

## 终点姿态:放开 pitch / yaw,锁住 roll

真机“右臂末端抵达最近的手机上空30厘米处”:舒适姿态 + via_then_rrt 一次到位(误差 0 mm),但轨迹关节总行程 387.7°,手腕绕工具轴乱转。原因:舒适姿态的 roll 用“基面法向”定,和当前手腕无关。

- [goal_pose.py](harness/motion/goal_pose.py) 新 `roll_locked` / `rotation_from_to` / `roll_change_deg`:工具轴(Z)从当前方向最小转动到新方向,X、Y 跟着带过去,不自转;comfort 和 free 都改用它(free:先只管位置找工具轴方向,再锁 roll 重解,解不出才退回并报告转了多少)。
- 新参数 `wrist_roll_deg`([gateway](harness/gateway.py) `_roll_arg`,`approach_object` / `move_arm_end`,−180~180):用户明确要转手腕才填;`move_arm_end` 可以只给它(原地转手腕);和 `rotate_deg` 二选一。keep 策略下也按它转。
- 离线(右臂、find 腰姿态、31 个点):comfort 自转中位 98° → 0°,关节总变化 308° → 252°,能到的点数不变(21);free 31/31、自转 0°。真机那次目标:自转 −121° → 0°。详见 [docs/goal_pose.md](docs/goal_pose.md#roll-锁住放开-pitch--yaw手腕不自转)。
- 测试:`test_roll_locked_only_tilts_tool_axis`、`test_free_and_comfort_keep_roll`、候选几何里加“不自转”检查、`test_wrist_roll_argument`。原限位下全过;现配置仍是那 3 个限位测试失败。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real "右臂末端抵达最近的手机上空30厘米处"
python3 scripts/run_task.py --real "右手手腕转30度"          # → move_arm_end(arm=right, wrist_roll_deg=30)
source /opt/ros/humble/setup.bash && python3 -m pytest tests/test_goal_pose.py -q
```

## “让夹爪竖直朝下”:模型不该自己算欧拉角 → `tool_axis`

现象(trace `traces/20261112_233245.jsonl`):“右臂往回收收让夹爪竖直朝下” → 模型 `move_arm_end(offset_cm=[-10,-5,0], rotate_deg=[0,0,36.4])`,想绕 z 转掉 yaw 来“朝下”。执行成功,但工具轴(末端 Z)离竖直朝下转前 77.9°、转后还是 77.9°(绕竖直轴转改不了倾斜);模型还说“只偏约 20°”,也算错了。

原因:“朝下”要换算成 base_link 欧拉角,大模型算不对。

改动:

- 新参数 `tool_axis`(`move_arm_end`、`approach_object`):`down / up / forward / backward / left / right`,工具轴朝 base_link 的这个方向,roll 用 `roll_locked` 锁住(不自转,可再加 `wrist_roll_deg`)。可以只给它(原地转朝下)。和 `rotate_deg` 二选一。
- `get_robot_state` 和 `move_arm_end` 结果里加 `tool_axis`(末端 Z 在 base_link 的方向)和 `tool_tilt_from_down_deg`(0 = 正朝下),模型直接看,不用算。
- 工具说明:`rotate_deg` 只在用户给具体转角时用;系统提示加“让夹爪朝下/朝前用 tool_axis,不要自己算欧拉角”。
- 后端 [`plan_arm_to(..., tool_axis=)`](harness/backends/ros.py) 新策略 `axis`;测试 `test_tool_axis_down`、`test_robot_state_reports_tool_tilt`,原限位下 109 个全过。

离线核对(用日志里的真实腰、右臂关节角,FK 末端 = 日志值,对得上):夹爪朝下在原位置 (0.798, −0.078, 1.012) IK 无解;往回收 10 cm 的 (0.698, −0.128, 1.012) 有解(离竖直 0.2°、自转 0.1°),但要大换构型(j3 −145°、j5 −151°),直线走不过去,要靠 via_then_rrt。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real --plan-only "右臂往回收10厘米,让夹爪竖直朝下"     # 先看规划
python3 scripts/run_task.py --real "右臂往回收10厘米,让夹爪竖直朝下"
python3 -m pytest tests/test_motion_tools.py -q -k "tool_axis or tool_tilt"
```

## “让夹爪竖直朝下”:原地转不到就自己后撤着转

用户:不该说“往回收 10 厘米”,撤多少由机器人自己判断——转不到就先转一些,再往后撤继续往下转。

- [goal_pose.py](harness/motion/goal_pose.py) 新 `tilt_toward`(工具轴往目标方向转几成,roll 锁住)、`tilt_search`(后撤 0、5、10…30 cm,每步先试转满,不行二分找最多转几成;转满就停,否则取转得最多的)。
- [`RosBackend._plan_tilt_retreat`](harness/backends/ros.py):后撤方向 = 末端→肩的水平方向;找到位姿后直线 → via_then_rrt 一次过去;报 `retreat_m`、`tilt_fraction`。
- [gateway](harness/gateway.py) `move_arm_end`:后端撤了就按实际终点再查工作范围;结果带 `retreat_m`、`tilt_fraction`。[configs/harness.yaml](configs/harness.yaml) `motion.tool_axis_retreat: {max_m: 0.30, step_m: 0.05}`。工具说明告诉模型“只给 tool_axis,撤多少不用你定”。
- 离线(日志那次的真实关节角):原地无解 → 后撤 10 cm 完全朝下,离竖直 1.0°、自转 0.2°,0.26 s。
- 测试:`test_tool_axis_retreats_by_itself`、`test_retreat_target_still_checked_against_workspace`、`test_tilt_search_retreats_until_gripper_points_down`;原限位下 112 个全过,现配置仍是那 3 个限位测试失败。**真机没跑**。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real --plan-only "右臂让夹爪竖直朝下"
python3 scripts/run_task.py --real "右臂让夹爪竖直朝下"
source /opt/ros/humble/setup.bash && python3 -m pytest tests/test_goal_pose.py -q
```

## “让右夹爪指向我的手”:模型去问颜色

现象(trace `traces/20261112_234127.jsonl`):模型没调任何工具,反问“您的手是什么颜色”,还问“指向”是放上方还是朝前。

原因:① 系统提示里 find_objects 的例子只有“手机、杯子、笔”,模型以为人的手只能按颜色定位;② 没有“指向”这个动作。

改动:

- 新工具 `point_at_object(object_id, arm)`([gateway `_point_at_object`](harness/gateway.py)):末端位置不动,工具轴转到“末端→物体”方向(roll 锁住),转不到就后撤着转;离物体 < 0.15 m 拒绝(`motion.point_min_distance_m`);执行后报 `pointing_error_deg`。
- [系统提示](harness/loop.py):说了物体名(包括人的手、脸)就用 find_objects(“我的手”→ hand),不要问颜色;locate_object 只在按颜色说时用;“指向”用 point_at_object。`find_objects` 的 query 例子加 hand、person。
- 录的帧上 SAM 3 `hand` 找到两只手(0.79、0.74)。
- 测试:原地转向手、太近拒绝、整条“让右夹爪指向我的手”;原限位下 115 个全过,现配置仍是那 3 个限位测试失败。**真机没跑**。

命令:

```bash
# 终端 1
cd ~/sam3_service && .venv/bin/python server.py
# 终端 2
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/run_task.py --real --plan-only "让右夹爪指向我的手"
python3 scripts/run_task.py --real "让右夹爪指向我的手"
```

## 长序任务中途“腰关节反馈过期”+ 夹爪位置为什么读不到

现象(trace `traces/20261112_234746.jsonl`,“和你的右夹爪和我的右手击掌”):`find_objects(hand)` → `get_robot_state`(第一次连 GDK,打了一行 `E... No end state message found`)→ `point_at_object` 报 `关节反馈不可用:过期 [idx01~05 腰关节]` → 模型转去开夹爪,任务没完成。

排查:

1. 怀疑 GDK 在同一进程里起 DDS / 阻塞时占着 GIL,卡住 ROS 回调。实测**不是**:单独订阅 `/joint_states`,GDK 初始化前 67 Hz、后 100 Hz;按 harness 结构起 RosBackend,第一次 `gripper_state`(3.1 s)期间关节反馈最大年龄 80 ms,之后 11 ms,监视线程最长被卡 87 ms。复现不出来,更像 `/joint_states` 本身断了一下;`ArmMotion._fresh` 只容忍 0.5 s,一断就报错。
2. 夹爪位置读不到:`/gdk/joint_state`(22 个关节)和 `/joint_states`(30 个)里都**没有夹爪关节**;GDK `get_end_state()` 靠 HAL 的“末端状态”消息,这台机器人不发。是机器人端的事,我们改不了。每读一次阻塞约 2 s,并打一行 E 日志(`GLOG_minloglevel` 环境变量也关不掉)。

改动:

- [arm_motion.py](harness/motion/arm_motion.py) `_fresh`:反馈过期先等最多 `JOINT_WAIT = 1.0` s,还不行才报错,报错里写每个关节过期了几秒。(这是从 agi@g2_robot 复制来的文件,这处是 robot_harness 自己加的。)
- [gripper_gdk.py](harness/backends/gripper_gdk.py):默认不读 `get_end_state`([harness.yaml](configs/harness.yaml) `gripper.read_position: false`),E 日志和 2 s 阻塞都没了;结果 msg 改成“这台机器人不回报夹爪开合位置,没有到位确认,按成功处理”。真机 `gripper_cli.py status` 不再打 E 日志,总用时 2.5 s(含 GDK 初始化)。
- 测试:原限位下全过;`tests/test_motion.py`(运控)16 个全过。

命令:

```bash
source ~/agi@g2_robot/env_g2.sh
source ~/.cache/agibot/app/env.sh
cd ~/robot_harness
python3 scripts/gripper_cli.py status                   # 不再有 "No end state" 那行
python3 scripts/run_task.py --real "和你的右夹爪和我的右手击掌"
```

## 文档整理

- 新建 [docs/index.md](docs/index.md)(文档目录)、[sam3_service.md](docs/sam3_service.md)、[perception_plan.md](docs/perception_plan.md)、[object_handles.md](docs/object_handles.md)。
- [docs/media/harness_overview.png](docs/media/harness_overview.png):一张图画出全部文件夹、文件、主要函数、数据流和 [harness_guide](docs/harness_guide.md) 的时序图,由 [make_overview.py](docs/media/make_overview.py) 生成。
- `~/sam3_service/README.md` 精简成启动命令,细节都挪到 [docs/sam3_service.md](docs/sam3_service.md)。

## 补记:learning/ 手搓教学(11.10~11.11)

[learning/stage1_llm/](learning/stage1_llm/) 从零写了对话 → 工具调用 → 循环 → gateway;[learning/stage2_ros/](learning/stage2_ros/README.md) 打通 ROS:

- [`ros_io.py`](learning/stage2_ros/ros_io.py) 的 `RosIO`:订阅 `/joint_states`(BEST_EFFORT,要用 `qos_profile_sensor_data`)和 `/robot_description`(TRANSIENT_LOCAL),tf2 查位姿,MoveIt `/check_state_validity` 查碰撞。
- [`g2_kinematics.py`](learning/stage2_ros/g2_kinematics.py) 自己写的正运动学,真机上和 TF 差 0.00 mm。
- [`check_ros.py`](learning/stage2_ros/check_ros.py) 分 5 步验证;[`fake_robot.py`](learning/stage2_ros/fake_robot.py) 拒绝在 domain 4 发布。
- 改名:`config.py`/`kinematics.py`/`llm.py`/`tools.py` 和库名太像,读代码时误以为是库 → 改成 `g2_config`、`g2_kinematics`、`deepseek_api`、`llm_tools`。也去掉了 `with ... as`、`lambda`,改成 try/finally 和普通 if。

# 2026.11.11

## 控制腰、头、双臂 + 手臂绕障(和 pose_ab_search 同一套)

问题:模型只能让左臂末端靠近物体,头、腰、右臂都动不了。

`agi@g2_robot` 的 `ros2 run genie_robot_agibot_g2_motion pose_ab_search` 用的是 `ArmMotion.plan_detour` + 默认规划器 `via_then_rrt`。对比后确认 `harness/motion/` 里的 `arm_motion.py`、`kin.py`、`algorithms.py`、`planners/` 和源码逐行一致(只有 import 路径不同),算法不用再拷,要做的是把它变成模型能调用、且经过 gateway 的工具。

| 新工具 | 走哪条规划 | gateway 守的底线 |
|---|---|---|
| `get_robot_state` | 只读 | — |
| `move_arm_end(arm, offset_cm, rotate_deg)` | `plan_cartesian` 直线,不行 `plan_detour`(via_then_rrt) | 只收相对偏移;两臂各自工作范围;单次 ≤0.60m、≤90° |
| `move_joints(group, deltas_deg / angles_deg)` | `WholeBody.plan`:关节直线 + 整机碰撞检查,会撞再 `JointDetour` | URDF 限位留 0.02rad;单次每关节 腰 45° / 头 90° / 臂 120° |
| `go_to_pose(name, groups)` | 同上 | 只能选 `poses.yaml` 里的位姿 |
| `approach_object` 加 `arm` | 原来就是直线 + 绕障 | 不变 |

- 所有运动共用:先规划 → 检查 → 只规划模式到此为止 → 人按 `y` → 执行;每条指令最多 5 次运动(原来 1 次)。
- 用户要求 gateway 不要太严:碰撞交给规划器,gateway 只挡单位写错、目标明显不合理。`max_path_dip_m` 默认 `null` 不检查。
- **注意**:`use_head_cloud:=false` 启动时 MoveIt 没有 octomap,规划器只知道机器人自身和 planning scene 里的碰撞体,桌子不在里面。要让绕障避开桌子,得开点云或往 planning scene 里加桌面碰撞体;或者把 `max_path_dip_m` 设成 0.05 兜底。
- `WholeBody.goto` 拆成 `plan()` + `run()`(`goto` 保留),规划和执行之间才能让人确认;`run()` 发现机器人离开规划起点 >0.05rad 就拒绝执行。
- 第一版工具说明没写关节含义,模型把“腰往前弯 90 度”平均分给腰的 5 个关节(含侧倾、转腰),“头往左转”也转错关节。从 URDF 读出轴向写进 `move_joints` 说明(腰 1~3 绕 y 俯仰、4 绕 x、5 绕 z;头 1 绕 z、2 绕 x、3 绕 y)后,假机器人上 3 条指令都选对了关节。
- 姿态换算放在新文件 `harness/geometry.py`(纯 numpy,rpy 约定和 `algorithms.py` 一致)。
- 测试:`tests/test_motion_tools.py` 21 个(假机器人);全部 64 个通过。`WholeBody.plan/run` 在隔离的 ROS domain 77 里用打桩的碰撞服务验证了:直线规划、会撞时返回原因、已在目标、规划后被挪动拒绝执行、控制器不在线。**真机未验证**,先用 `--plan-only`。
- `docs/harness_guide.md` 第 7~9 章重写(原来的摘录已经和代码对不上),全部 92 个行号链接、51 段代码摘录和当前代码逐字核对过。

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
