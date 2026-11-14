"""生成 robot_harness 全景图:左边文件/函数/数据流地图,右边 harness_guide.md 第 2 章的时序图。一张 SVG。"""
import html
import sys

W, H = 3640, 2800
SANS = "Noto Sans CJK SC, Noto Sans CJK TC, sans-serif"
MONO = "DejaVu Sans Mono, Noto Sans CJK SC, monospace"
LINE = 21          # 函数行高
HEAD = 32          # 卡片标题高

C = {  # 颜色
    "bg": "#ffffff", "ink": "#1f2328", "muted": "#57606a", "grid": "#d0d7de",
    "entry": ("#fff8e6", "#d4a72c"), "core": ("#eef6ff", "#2f6fbd"), "tools": ("#f0f9f1", "#2f8a3e"),
    "gw": ("#fff0f0", "#c4302b"), "backend": ("#f6f0ff", "#7a4cc2"), "motion": ("#eef9f9", "#1d8a8a"),
    "percep": ("#fdf2fa", "#b0418a"), "cfg": ("#f5f5f5", "#6e7781"), "ext": ("#fffbe6", "#9a6700"),
    "ros": ("#fff4e8", "#d1661a"), "test": ("#f5f5f5", "#8c959f"),
}
ARROW = {"call": "#1f2328", "data": "#2f6fbd", "ros": "#d1661a", "dev": "#8c959f", "human": "#c4302b"}

out = []
boxes = {}


def esc(s):
    return html.escape(str(s), quote=True)


def text(x, y, s, size=15, weight="normal", fill=None, family=SANS, anchor="start"):
    out.append(f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" font-weight="{weight}" '
               f'fill="{fill or C["ink"]}" text-anchor="{anchor}">{esc(s)}</text>')


def card(key, x, y, w, title, lines, style, note=None):
    """一个文件卡片:标题(文件名)+ 每行一个函数/说明。返回高度。"""
    fill, stroke = C[style]
    h = HEAD + LINE * len(lines) + 12
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{HEAD - 4}" rx="8" fill="{stroke}"/>')
    out.append(f'<rect x="{x}" y="{y + HEAD - 12}" width="{w}" height="8" fill="{stroke}"/>')
    text(x + 10, y + 21, title, 16, "bold", "#ffffff", MONO)
    if note:
        text(x + w - 10, y + 21, note, 13, "normal", "#ffffff", SANS, "end")
    for i, line in enumerate(lines):
        color = C["muted"] if line.startswith("·") else C["ink"]
        text(x + 10, y + HEAD + 16 + i * LINE, line, 14, "normal", color, MONO)
    boxes[key] = (x, y, w, h)
    return h


def group(x, y, w, h, title, style, dashed=True):
    fill, stroke = C[style]
    dash = ' stroke-dasharray="8 5"' if dashed else ""
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="none" stroke="{stroke}" stroke-width="2"{dash}/>')
    tw = sum(19 if ord(c) > 127 else 11 for c in title) + 24
    out.append(f'<rect x="{x + 14}" y="{y - 14}" width="{tw}" height="26" rx="6" fill="#ffffff"/>')
    text(x + 24, y + 5, title, 18, "bold", stroke, MONO)


def anchor(key, side, t=0.5):
    x, y, w, h = boxes[key]
    return {"top": (x + w * t, y), "bottom": (x + w * t, y + h),
            "left": (x, y + h * t), "right": (x + w, y + h * t)}[side]


def arrow(points, kind="call", label=None, lx=None, ly=None, both=False, dashed=False, lanchor="start", lsize=14):
    color = ARROW[kind]
    pts = " ".join(f"{x},{y}" for x, y in points)
    dash = ' stroke-dasharray="7 5"' if dashed or kind == "dev" else ""
    start = f' marker-start="url(#a_{kind}_s)"' if both else ""
    out.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2.2"{dash} '
               f'marker-end="url(#a_{kind})"{start}/>')
    if label:
        for i, part in enumerate(label.split("\n")):
            yy = ly + i * 18
            width = sum(15 if ord(c) > 127 else 8.2 for c in part) + 10
            bx = lx - 5 if lanchor == "start" else lx - width + 5
            out.append(f'<rect x="{bx}" y="{yy - 14}" width="{width}" height="19" rx="3" fill="#ffffff" opacity="0.92"/>')
            text(lx, yy, part, lsize, "normal", color, MONO, lanchor)


def defs():
    out.append("<defs>")
    for k, c in ARROW.items():
        out.append(f'<marker id="a_{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" '
                   f'orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>')
        out.append(f'<marker id="a_{k}_s" viewBox="0 0 10 10" refX="1" refY="5" markerWidth="8" markerHeight="8" '
                   f'orient="auto"><path d="M10,0 L0,5 L10,10 z" fill="{c}"/></marker>')
    out.append("</defs>")


# ===================================================================== 画布
out.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
defs()
out.append(f'<rect width="{W}" height="{H}" fill="{C["bg"]}"/>')
text(40, 52, "robot_harness 全景:文件夹 · 文件 · 函数 · 数据流", 34, "bold")
text(40, 84, "智元 G2 + DeepSeek 工具调用。左:每个方框是一个文件,白字是文件名,下面是主要函数;箭头上写的是传递的东西。"
             "右:一条真实指令的完整时序(docs/harness_guide.md 第 2 章)。", 17, "normal", C["muted"])

# 图例
lgx, lgy = 2420, 40
for i, (k, name) in enumerate([("call", "函数调用"), ("data", "返回 / 数据"), ("ros", "ROS 话题 / 服务 / action"),
                               ("human", "人工确认"), ("dev", "开发 / 测试用")]):
    x = lgx + i * 240 - (0 if i < 4 else -20)
    arrow([(x, lgy + 20), (x + 50, lgy + 20)], k)
    text(x + 58, lgy + 25, name, 15, "normal", ARROW[k])

# ===================================================================== 第 0 行:人、根目录、configs、tests
Y0 = 140
card("user", 60, Y0, 300, "用户 / 终端", ["中文指令:", "  左手抵达红色积木上空", "按 y 确认每次运动", "守着急停"], "ext")
group(390, Y0 + 14, 600, 196, "项目根目录 /", "cfg")
card("root", 410, Y0 + 34, 560, "根目录文件", [
    ".env        DEEPSEEK_API_KEY(gitignore)",
    "run.txt     真机启动与运行命令",
    "README.md   项目介绍    devlog.md 开发日志",
    ".gitignore",
    "· 每个目录都有 README.md"], "cfg")
group(1020, Y0 + 14, 900, 196, "configs/", "cfg")
card("hy", 1040, Y0 + 34, 300, "harness.yaml", ["llm:    base_url model key", "agent:  max_steps 8", "motion: 速度 工作范围 上限", "vision: 帧数 大小阈值"], "cfg")
card("cy", 1350, Y0 + 34, 260, "colors.yaml", ["颜色名 → HSV 范围", "red green ... 紫色杯子", "hsv_pick.py 写入"], "cfg")
card("py", 1620, Y0 + 34, 280, "poses.yaml", ["位姿名 → 22 个关节角", "find / look / zero", "groups: body head 两臂"], "cfg")
group(1950, Y0 + 14, 430, 196, "tests/  (pytest,不需要 ROS)", "test")
card("t", 1970, Y0 + 34, 390, "3 个测试文件", [
    "test_objects.py      27 个:定位 靠近 颜色",
    "test_motion_tools.py 21 个:手臂 关节 位姿",
    "test_motion.py        8 个:IK 绕障(需 URDF)",
    "ScriptedLLM:假模型按剧本回复"], "test")

# ===================================================================== 第 1 行:scripts/ + DeepSeek
Y1 = 470
group(40, Y1, 1880, 230, "scripts/  (命令行入口)", "entry")
card("run_task", 60, Y1 + 24, 480, "run_task.py", [
    "main()      解析 --real --plan-only",
    "load_env()  .env → 环境变量",
    "load_poses()   poses.yaml",
    "run_one(task)  每条指令一个新 Gateway",
    "ask_user(prompt) → 输入 y ?",
    "· 写 traces/<时间>.jsonl"], "entry", "主入口")
card("goto_pose", 560, Y1 + 24, 420, "goto_pose.py", [
    "main()  全身到录好的位姿",
    "load_poses()  joint_limits(urdf)",
    "→ WholeBody.goto + JointDetour",
    "· 不经过大模型"], "entry")
card("motion_cli", 1000, Y1 + 24, 440, "motion_cli.py", [
    "main()  手臂交互命令行",
    "move / line / goto / detour",
    "_target() _confirm_and_run()",
    "→ ArmMotion(不经过大模型)"], "entry")
card("hsv_pick", 1460, Y1 + 24, 440, "hsv_pick.py", [
    "main()  interactive()",
    "sample_at(像素) → HSV",
    "save() → save_color(colors.yaml)",
    "· 点选取色,左键点物体"], "entry")
card("deepseek", 1960, Y1 + 24, 420, "DeepSeek API(外部)", [
    "POST /chat/completions",
    "model: deepseek-flash",
    "输入 messages + tools",
    "返回 content / tool_calls"], "ext")

# ===================================================================== 第 2 行:harness/ 核心
Y2 = 860
group(40, Y2, 2340, 420, "harness/  (核心:循环 · 工具说明 · 关卡)", "core")
card("loop", 60, Y2 + 30, 360, "loop.py", [
    "run_agent(task, llm, gateway,",
    "          tools, max_steps, trace)",
    "  ├ log(kind, data) → trace",
    "  └ 循环直到没有 tool_calls",
    "_execute(call, gateway)",
    "  json.loads(arguments)",
    "SYSTEM  系统提示词"], "core", "Agent 循环")
group(440, Y2 + 30, 560, 250, "harness/tools/", "tools")
card("objects", 455, Y2 + 52, 260, "objects.py", [
    "make_tools(颜色, 位姿)",
    "_object_tools()",
    "  locate_object",
    "  get_left_end_pose",
    "  approach_object",
    "_tool(name, desc, props)"], "tools")
card("tmotion", 725, Y2 + 52, 260, "motion.py", [
    "make_motion_tools(位姿)",
    "  get_robot_state",
    "  move_arm_end",
    "  move_joints",
    "  go_to_pose",
    "GROUP_TEXT 关节含义"], "tools")
card("gateway", 1020, Y2 + 30, 560, "gateway.py  class Gateway", [
    "execute(name, args) → 按名字分派 handlers",
    "_locate(color)      → self.target 存坐标",
    "_end_pose()  _robot_state()      只读",
    "_approach(standoff, arm)  目标=物体+上方",
    "_move_arm_end(arm, offset_cm, rotate_deg)",
    "_move_joints(group, deltas/angles_deg)",
    "_go_to_pose(name, groups)",
    "_plan_and_run_joints(target) 限位检查",
    "_check_workspace()  _no_moves_left()",
    "_confirm_and_run(plan, prompt, info)",
    "  只规划?→ 停;confirm?→ execute",
    "· 状态:target  target_time  moves"], "gw", "关卡")
card("llm", 1980, Y2 + 30, 380, "llm.py  class DeepSeekLLM", [
    "__init__(base_url, model,",
    "         api_key_env, thinking)",
    "chat(messages, tools) → reply",
    "  urllib POST,只挑需要的字段",
    "  保留 reasoning_content"], "core")
card("geometry", 1600, Y2 + 30, 360, "geometry.py", [
    "rpy_deg_to_matrix(rpy)",
    "matrix_to_rpy_deg(R)",
    "rotate_rpy_deg(rpy, rotate)",
    "rotation_between_deg(a, b)"], "core")
card("colors", 1600, Y2 + 195, 360, "colors.py", [
    "load_colors(path) → {名: 范围}",
    "save_color(path, 名, 范围)",
    "range_from_samples(样本)"], "core")

# ===================================================================== 第 3 行:backends/
Y3 = 1400
group(40, Y3, 2340, 320, "harness/backends/  (同一套方法,两种实现)", "backend")
card("fake", 60, Y3 + 30, 520, "fake.py  class FakeBackend", [
    "locate_object(color, ranges)",
    "end_pose(arm)   left_end_xyz()",
    "joint_state()   joint_limits()",
    "plan_arm_to(arm, target, rpy)",
    "plan_left_to(target)",
    "plan_joints(target, speed)",
    "execute(plan) → 改自己的状态",
    "· executed[] 测试用:确认真的没动"], "backend", "不需要 ROS")
card("ros", 600, Y3 + 30, 1100, "ros.py  class RosBackend", [
    "__init__(vision, speed, poses): rclpy 节点 + ArmMotion + WholeBody + SensorNode,后台线程 spin",
    "_wait_until_ready()  没有 /joint_states 或 move_group 就拒绝启动",
    "locate_object(color, ranges)  取 5 帧 → find_objects → merge_frames → 存 block_xyz",
    "_save_debug_image() → traces/last_object.jpg    _publish_tf() → TF target_object",
    "end_pose(arm) → {xyz_m, rpy_deg}    joint_state()    joint_limits()  URDF",
    "plan_arm_to(arm, target, rpy)  plan_cartesian 直线,不行 → plan_detour(via_then_rrt)",
    "_lowest_z(arm, path)  路径最低点    slow_down(plan, speed)  放慢到 5cm/s",
    "plan_joints(target, speed)  WholeBody.plan + JointDetour",
    "execute(plan)  kind=arm → ArmMotion.execute;kind=joints → WholeBody.run",
    "close()  取消运动 → 停 spin → 销毁节点"], "backend", "真机")
card("color_object", 1720, Y3 + 30, 640, "color_object.py", [
    "color_mask(bgr, ranges) → 掩码",
    "find_objects(bgr, depth, points,",
    "             fx, ranges, cfg) → 候选",
    "merge_frames(每帧, 最少帧数, 最大偏差,",
    "             左手位置) → 离左手最近的物体",
    "· 纯 numpy / OpenCV,不需要 ROS"], "backend")

# ===================================================================== 第 4 行:motion/ + perception/
Y4 = 1850
group(40, Y4, 1840, 640, "harness/motion/  (复制自 agi@g2_robot ecdfb5d:运动学 · 规划 · 执行)", "motion")
card("arm_motion", 60, Y4 + 30, 470, "arm_motion.py  class ArmMotion", [
    "plan_cartesian(arm, p, R) 积分笛卡尔直线",
    "plan_detour(arm, p, R, planner)  绕障",
    "plan_pose(arm, p, R)  IK + 五次多项式",
    "execute(Plan) → JTC action,等停稳",
    "pose(arm) → (p, R)   q_arm(arm)",
    "valid(arm, q)   path_clear(arm, path)",
    "obstacles_root()  octomap + 碰撞体",
    "cancel()  wait_settled()",
    "class Plan: path traj note summary()"], "motion", "手臂")
card("whole_body", 545, Y4 + 30, 440, "whole_body.py  class WholeBody", [
    "plan(target, speed, detour)",
    "   → JointPlan / 失败原因",
    "run(JointPlan) → 4 个 JTC 同时发",
    "goto() = plan + run",
    "snapshot()  _valid()  check_path()",
    "cancel()   _settle() 实测到位",
    "class JointPlan: qs Ts summary()"], "motion", "腰头双臂")
card("joint_detour", 1000, Y4 + 30, 330, "joint_detour.py", [
    "class JointDetour",
    "  plan(q0, target) → 折线",
    "  _via()  经录好位姿中转",
    "  _rrt()  关节 RRT-Connect",
    "  _shortcut()  捷径平滑",
    "path_note()"], "motion")
card("kin", 1345, Y4 + 30, 250, "kin.py", [
    "class ChainKin",
    "  fk(q)  jacobian(q)  ik()",
    "arm_kin(urdf, arm)",
    "body_kin(urdf)  腰链",
    "dls_pinv()"], "motion")
card("algorithms", 1610, Y4 + 30, 255, "algorithms.py", [
    "class ArmSolver",
    "  solve() 积分笛卡尔",
    "  ik()  lm()  pull_near()",
    "polish()  path_sane()",
    "rpy_to_mat()  interp()"], "motion")
group(60, Y4 + 300, 880, 250, "planners/  (绕障规划器,默认 via_then_rrt)", "motion")
card("via_then_rrt", 75, Y4 + 322, 280, "via_then_rrt.py", ["ViaThenRRT.plan(req)", "  先模板,不通再 RRT"], "motion")
card("via_points", 365, Y4 + 322, 280, "via_points.py", ["try_templates()", "  V / 梯形 / U 型绕行"], "motion")
card("ee_rrt", 655, Y4 + 322, 270, "ee_rrt.py", ["EERRT.plan()  rrt()", "  末端空间 RRT"], "motion")
card("edges", 75, Y4 + 432, 280, "_edges.py", ["EdgeChecker", "  edge() shortcut() direct()"], "motion")
card("types", 365, Y4 + 432, 280, "_types.py  __init__.py", ["PlanRequest  PlanResult", "Planner  DEFAULT"], "motion")
card("registry", 655, Y4 + 432, 270, "_registry.py", ["register()  discover()", "AlgorithmSpec"], "motion")
group(960, Y4 + 300, 560, 250, "ik_lab/  (轨迹计时 · 终点精修)", "motion")
card("planning", 975, Y4 + 322, 260, "planning.py", ["make_trajectory()", "plan_motion()"], "motion")
card("ikkin", 1245, Y4 + 322, 260, "kinematics.py", ["solve_pose()", "rotation_vector()"], "motion")
card("pose_task", 975, Y4 + 432, 260, "pose_task.py", ["orientation_residual()", "task_nullspace()"], "motion")
card("adaptive", 1245, Y4 + 432, 260, "algorithms/_adaptive_ik.py", ["solve_adaptive()", "  自适应 LM"], "motion")
card("joints", 1540, Y4 + 300, 325, "joints.py", ["GROUPS: 4 组 → JTC 名 + 关节", "  body head left_arm right_arm", "parse_groups(text)"], "motion")
card("octomap", 1540, Y4 + 420, 325, "octomap_util.py", ["occupied_centers(octomap)"], "motion")
card("voxel", 1540, Y4 + 487, 325, "voxel_util.py", ["quat_to_mat()  dilate()"], "motion")

group(1910, Y4, 470, 640, "harness/perception/  (头部 RGBD)", "percep")
card("rgbd_source", 1930, Y4 + 30, 430, "rgbd_source.py", [
    "class RosSource(sensor)",
    "  next() → (bgr, depth_mm,",
    "            T_base_head, stamp)",
    "  按时间戳对齐 彩色/深度/头部 TF",
    "class DatasetSource  离线回放"], "percep")
card("sensor_node", 1930, Y4 + 205, 430, "sensor_node.py", ["class SensorNode", "  open()  close()  destroy()", "  独立线程 spin 相机订阅"], "percep")
card("scene3d", 1930, Y4 + 335, 430, "scene3d.py", ["class Rgbd", "  points_base(depth, T)", "  → 每个像素的 base_link 坐标"], "percep")
card("calib", 1930, Y4 + 465, 430, "calib.py", ["CALIB  相机内外参", "read_gdk_calib()"], "percep")

# ===================================================================== 第 5 行:ROS 2
Y5 = 2600
group(40, Y5, 2340, 180, "ROS 2", "ros", dashed=False)
text(60, Y5 + 160, "hardware.launch.py end_effector:=true use_head_cloud:=false  (由人启动,AI 不启动;use_head_cloud:=false 时没有 octomap,规划器看不到桌子)", 15, "normal", C["ros"][1], MONO)
chips = [
    ("js", "/joint_states", "JointState  BEST_EFFORT"),
    ("rd", "/robot_description", "URDF  TRANSIENT_LOCAL"),
    ("sv", "/check_state_validity", "MoveIt 整机碰撞检查"),
    ("ps", "/get_planning_scene", "octomap + 碰撞体"),
    ("jtc", "4 × FollowJointTrajectory", "body/head/left_arm/right_arm"),
    ("cam", "/gdk/camera/head_color", "head_depth + /gdk/tf"),
    ("tf", "/tf", "target_object(本项目发布)"),
]
cx = 60
for key, name, sub in chips:
    card(key, cx, Y5 + 30, 318, name, [sub], "ros")
    cx += 326
text(2210, Y5 + 165, "→ 智元 G2", 18, "bold", C["ros"][1])

# ===================================================================== 箭头
# 人 → run_task
arrow([anchor("user", "bottom", 0.4), (180, Y1 + 24)], "human", "task 文本", 190, Y1 - 30)
# .env / configs → run_task
arrow([anchor("root", "bottom", 0.15), (494, Y1 + 24)], "data", "API key", 502, Y1 - 50)
arrow([anchor("hy", "bottom", 0.3), (1130, Y1 - 60), (420, Y1 - 60), (420, Y1 + 24)], "data", "cfg(dict)", 1000, Y1 - 66)
arrow([anchor("cy", "bottom", 0.3), (1428, Y1 - 40), (450, Y1 - 40), (450, Y1 + 24)], "data", "colors", 1300, Y1 - 46)
arrow([anchor("py", "bottom", 0.3), (1704, Y1 - 20), (480, Y1 - 20), (480, Y1 + 24)], "data", "poses", 1580, Y1 - 26)
# run_task → loop / gateway / tools
arrow([anchor("run_task", "bottom", 0.15), (132, Y2 + 30)], "call",
      "run_agent(task, llm,\ngateway, tools, trace)", 140, Y2 - 92)
arrow([anchor("run_task", "bottom", 0.55), (324, Y2 - 30), (1150, Y2 - 30), (1150, Y2 + 30)], "call",
      "Gateway(backend, cfg, colors,\nconfirm=ask_user, plan_only, poses)", 700, Y2 - 72)
arrow([anchor("run_task", "bottom", 0.85), (468, Y2 - 10), (585, Y2 - 10), (585, Y2 + 52)], "call",
      "make_tools(colors, poses)", 600, Y2 - 14)
# 确认:gateway → ask_user → 人
arrow([anchor("gateway", "top", 0.85), (1496, Y2 - 105), (520, Y2 - 105), anchor("run_task", "bottom", 0.96)],
      "human", "confirm(规划摘要) → ask_user() → 人按 y ?", 1000, Y2 - 112)
# tools → loop
arrow([anchor("objects", "left", 0.2), (420, Y2 + 52 + 34)], "data", "TOOLS", 424, Y2 + 262)
# loop ↔ llm ↔ deepseek
arrow([anchor("loop", "bottom", 0.25), (150, Y2 + 380), (2170, Y2 + 380), anchor("llm", "bottom", 0.5)], "call",
      "llm.chat(messages, tools) → reply {content, tool_calls[id, name, arguments]}", 1040, Y2 + 372)
arrow([anchor("llm", "top", 0.5), anchor("deepseek", "bottom", 0.5)], "data",
      "HTTPS JSON\n(key 在 header)", 2185, Y2 - 70, both=True)
# loop → gateway
arrow([anchor("loop", "bottom", 0.75), (330, Y2 + 310), (1020, Y2 + 310)], "call",
      "gateway.execute(name, args) → result dict", 470, Y2 + 302)
# gateway → geometry
arrow([anchor("gateway", "right", 0.25), anchor("geometry", "left", 0.62)], "call")
# gateway → backend
arrow([anchor("gateway", "bottom", 0.5), (1300, Y3 + 30)], "call",
      "locate_object · end_pose · joint_state · joint_limits\nplan_arm_to · plan_joints → plan\nexecute(plan) → {ok, msg}",
      1312, Y3 - 74)
arrow([anchor("gateway", "bottom", 0.12), (1087, Y3 - 10), (320, Y3 - 10), (320, Y3 + 30)], "dev",
      "测试 / 无 --real", 360, Y3 - 16)
# ros → color_object / perception / motion
arrow([anchor("ros", "right", 0.12), anchor("color_object", "left", 0.12)], "call")
arrow([anchor("ros", "bottom", 0.88), (1568, Y4 - 30), (2145, Y4 - 30), (2145, Y4 + 30)], "call",
      "RosSource.next() → bgr, depth, T_base_head;points_base()", 1600, Y4 - 38)
arrow([anchor("ros", "bottom", 0.12), (732, Y4 - 60), (295, Y4 - 60), (295, Y4 + 30)], "call",
      "plan_cartesian / plan_detour → Plan;execute(Plan)", 120, Y4 - 66)
arrow([anchor("ros", "bottom", 0.4), (1040, Y4 - 30), (765, Y4 - 30), (765, Y4 + 30)], "call",
      "plan(target, speed, JointDetour) → JointPlan;run()", 780, Y4 - 36)
# motion 内部
arrow([anchor("arm_motion", "bottom", 0.4), (248, Y4 + 322)], "call", "PlanRequest → PlanResult", 258, Y4 + 285, lsize=13)
arrow([anchor("whole_body", "right", 0.3), anchor("joint_detour", "left", 0.3)], "call")
# scripts → 内部(不经过模型)
# ROS 通信
arrow([(219, Y5 + 30), (219, Y4 + 640)], "ros", "关节角", 229, Y4 + 680)
arrow([(545, Y5 + 30), (545, Y4 + 640)], "ros", "URDF", 555, Y4 + 680)
arrow([(871, Y4 + 640), (871, Y5 + 30)], "ros", "关节角 → valid?", 881, Y4 + 680, both=True)
arrow([(1197, Y4 + 640), (1197, Y5 + 30)], "ros", "octomap / 碰撞体", 1207, Y4 + 680, both=True)
arrow([(1523, Y4 + 640), (1523, Y5 + 30)], "ros", "轨迹目标 ↔ 结果", 1533, Y4 + 680, both=True)
arrow([(1960, Y5 + 30), (1960, Y4 + 640)], "ros", "彩色 + 深度 + 头部位姿", 1970, Y4 + 680)
arrow([(1520, Y3 + 30 + HEAD + 12 + LINE * 10), (1520, Y4 - 70), (2395, Y4 - 70), (2395, Y5 + 75), (2342, Y5 + 75)], "ros", "_publish_tf()", 2250, Y4 - 78)

# ===================================================================== 右边:时序图
SX0, SY0, SW = 2420, 130, 1200
group(SX0, SY0, SW, H - SY0 - 30, "时序:“左手抵达红色积木上空”(traces/20261109_015006.jsonl)", "gw", dashed=False)
parts = [("U", "用户"), ("L", "loop.py"), ("M", "DeepSeek"), ("G", "gateway.py"), ("R", "ros.py")]
px = {k: SX0 + 120 + i * 240 for i, (k, _) in enumerate(parts)}
top = SY0 + 40
bottom = H - 70
for k, name in parts:
    out.append(f'<rect x="{px[k] - 85}" y="{top}" width="170" height="44" rx="8" fill="#ffffff" stroke="#1f2328" stroke-width="2"/>')
    text(px[k], top + 29, name, 18, "bold", C["ink"], MONO, "middle")
    out.append(f'<line x1="{px[k]}" y1="{top + 44}" x2="{px[k]}" y2="{bottom}" stroke="#8c959f" stroke-width="1.5" stroke-dasharray="6 5"/>')
msgs = [
    ("U", "L", "左手抵达红色积木上空", False),
    ("L", "M", "system + user + 工具列表", False),
    ("M", "L", "调用 locate_object(color=red)", True),
    ("L", "G", 'execute("locate_object", {color: red})', False),
    ("G", "R", 'locate_object("red", HSV 范围)', False),
    ("R", "G", "xyz=(0.790, 0.097, 0.767)", True),
    ("G", "L", "结果", True),
    ("L", "M", "追加工具结果,再问", False),
    ("M", "L", "调用 get_left_end_pose()", True),
    ("L", "G", "execute(...)", False),
    ("G", "L", "xyz=(0.677, 0.392, 1.100)", True),
    ("L", "M", "再问", False),
    ("M", "L", "调用 approach_object()", True),
    ("L", "G", "execute(...)", False),
    ("G", "G", "算目标点 + 7 项检查", False),
    ("G", "R", 'plan_arm_to("left", 目标)', False),
    ("G", "U", "打印规划,等待 y", False),
    ("U", "G", "y", True),
    ("G", "R", "execute(plan)", False),
    ("R", "G", "到位,误差 0.1mm", True),
    ("L", "M", "再问", False),
    ("M", "L", "纯文字回答(没有工具调用)", True),
    ("L", "U", "完成……", True),
]
step = (bottom - top - 110) / len(msgs)
for i, (a, b, label, ret) in enumerate(msgs):
    y = top + 100 + i * step
    kind = "data" if ret else "call"
    if a == b and a == "G":
        x = px[a]
        arrow([(x, y), (x + 70, y), (x + 70, y + 28), (x + 6, y + 28)], "gw" if False else "human", None)
        text(x + 80, y + 18, f"{i + 1}. {label}", 16, "bold", ARROW["human"], MONO)
        continue
    if (a, b) in [("G", "U"), ("U", "G")]:
        kind = "human"
    x1, x2 = px[a], px[b]
    d = 6 if x2 > x1 else -6
    arrow([(x1, y), (x2 - d, y)], kind, dashed=ret)
    mid = (x1 + x2) / 2
    lab = f"{i + 1}. {label}"
    width = sum(15 if ord(c) > 127 else 8.4 for c in lab)
    lx = mid - width / 2
    lx = max(SX0 + 12, min(lx, SX0 + SW - width - 12))
    out.append(f'<rect x="{lx - 4}" y="{y - 24}" width="{width + 8}" height="20" rx="3" fill="#ffffff" opacity="0.9"/>')
    text(lx, y - 9, lab, 15, "normal", ARROW[kind], MONO)

out.append("</svg>")
open(sys.argv[1], "w", encoding="utf-8").write("\n".join(out))
print("written", sys.argv[1])
