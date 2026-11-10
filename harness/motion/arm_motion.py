#!/usr/bin/env python3
"""arm_motion.py —— G2 手臂运控执行层（ROS）：把 algorithms.py 的两个算法接到 G2 真机上。

链路：本节点 → /{left,right}_arm_group_controller/follow_joint_trajectory(JTC action)
      → genie_hardware/GenieG2ArmHardware → /gdk/joint_control → 机器人。
前提：hardware.launch.py 在跑（controller_manager + JTC + rsp + move_group）。

两种运动：
  cartesian_to()  积分笛卡尔走直线（sanofi）：末端沿直线到目标，构型连续；过 path_sane 防大回环。
                  终点再用自适应 LM 精修到 0.1mm。沿路径变速(cart_profile)：末端匀速 CART_SPEED，
                  单关节会超 URDF 速度限位 / CART_JOINT_ACCEL 的地方就地压末端速度；中途不停。
  pose_to()       多种子 IK 解终点（sanofi：取最近且整机不撞的分支，零空间拉近）+ LM 精修(genie)
                  → 起点/终点两点五次多项式(genie pose_ab 的计时)；直连会撞就末端绕障规划(planners/，默认 planners.DEFAULT)。
  plan_detour()   强制末端空间网格搜索：5cm 格子加权 A*，每段积分笛卡尔 + 整机碰撞(含 octomap)，捷径平滑成几段直线。
                  不用 OMPL——OMPL 在关节空间随机采样，会抡大圈。

合并来源（2026-11-02）：
  sanofi grasp_server：两个算法本体、执行后按关节反馈确认停稳。（OMPL 绕行已换成 planners/ 末端绕障，2026-11-02）
  genie pose_ab/planning：整机碰撞检查(不只本臂规划组)、路径每 2° 采一次碰撞、0.1mm 终点精度、
                          五次多项式计时、FollowJointTrajectory action(有执行结果、可取消)。

坐标约定：对外一律 base_link 系（x 前 y 左 z 上）。G2 手臂链根 arm_base_link 绕 X 转了 −90°，
  轴向跟"前左上"对不上，所以内部用【实测腰关节】的 FK(base_link→arm_base_link) 换算后再解。

安全：不发命令的情况 —— URDF/关节反馈没到、反馈超过 JOINT_MAX_AGE 没更新、JTC action 不在线、
  积分没到位、路径不过 path_sane、碰撞检查不过、规划后手臂已离开起点。
  碰撞服务不可用(move_group 没起)时只警告一次后放行，同 sanofi。执行中 Ctrl-C → 取消 JTC 目标。
"""
import math
import threading
import time

import numpy as np
from scipy.interpolate import CubicSpline
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from urdf_parser_py.urdf import URDF

from .kin import arm_kin, body_kin
from .algorithms import ArmSolver, path_sane, interp, polish, rotvec, STEP_JOINT_MAX, LM_ANG_W
from .ik_lab.planning import MotionSettings, make_trajectory
from . import planners
from .octomap_util import occupied_centers
from .voxel_util import quat_to_mat
from shape_msgs.msg import SolidPrimitive

ARMS = ("left", "right")
ACTION = {"left": "/left_arm_group_controller/follow_joint_trajectory",
          "right": "/right_arm_group_controller/follow_joint_trajectory"}

JOINT_MAX_AGE = 0.5          # 参与解算的关节反馈最大年龄(s)
# 关节点到点(pose_to)：genie pose_ab 的默认值
#   2026-11-02 提速：速度不再另设上限，直接用 URDF 限位(3.3~5.3rad/s)；加速度 0.8、加加速度 3.0 仍限
#   （1rad 的点到点 7.5s → ~2.7s，现在是加速度/加加速度说了算）
JOINT_SETTINGS = MotionSettings(velocity=math.inf, acceleration=0.8, jerk=3.0, travel_warning_deg=90.0)
# 笛卡尔直线(cartesian_to)：沿路径变速（cart_profile）。末端按 CART_SPEED 匀速，起止按 CART_ACCEL 加减速；
#   哪里单关节会超 URDF 速度限位 / CART_JOINT_ACCEL，就只在那一段把末端速度压下来。
CART_SPEED = 0.15            # m/s 末端巡航速度（平移）
CART_ACCEL = 0.3             # m/s² 末端加减速（0→0.15 用 0.5s、3.75cm）
CART_JOINT_ACCEL = 3.0       # rad/s² 单关节加速度上限（原五次多项式整条一个曲线：奇异点附近实测到 21）
CART_DS = 0.005              # 路径重采样间距（末端弧长，m）
CART_SMOOTH = 0.03           # 关节角沿末端弧长平滑的窗口（m）：去 IK 小步抖动 + 绕障拐角倒小圆角（末端偏几 mm，复查碰撞）
CART_DT = 0.05               # 发给 JTC 的轨迹点时间间隔（s）
CHECK_STEP = math.radians(2) # 路径碰撞检查：关节每变化这么多采一次（genie planning 同款）
SV_TIMEOUT = 1.0             # /check_state_validity 单次调用超时(s)
OBJ_SAMPLE = 0.025           # 碰撞体表面采样间距(m)：给末端粗筛当障碍点（粗筛每半格 2.5cm 查一次、离障 10cm 才拒）
POLISH_ITERS = 30            # 终点 LM 精修迭代上限（3mm → 0.1mm）
ACTION_WAIT = 2.0            # 等 action server / 接受目标的超时(s)
RESULT_MARGIN = 5.0          # 等执行结果的超时 = 轨迹时长 + 它
SETTLE_TOL = 0.02            # 执行成功后再确认：每个关节离终点 ≤ 它(rad)。G2 是 KP-KD，稳态误差实测 ~0.001rad
SETTLE_STILL = 0.4           # 且连续这么久不动(s)
SETTLE_TIMEOUT = 3.0


def _triangle_points(T, step):
    """三角形 (N,3,3) 表面按 step 间距撒点（重心坐标网格）。"""
    out = []
    for a, b, c in T:
        n = int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / step)) + 1
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        k = i + j <= n
        out.append(a + np.outer(i[k] / n, b - a) + np.outer(j[k] / n, c - a))
    return np.concatenate(out) if out else np.empty((0, 3))


_BOX_TRI = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                     [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])


def _primitive_points(prim, step):
    """SolidPrimitive 表面撒点（自身坐标系）；不认识的类型返回空。"""
    d = prim.dimensions
    if prim.type == SolidPrimitive.BOX:
        V = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)]) * np.asarray(d[:3])
        return _triangle_points(V[_BOX_TRI], step)
    if prim.type in (SolidPrimitive.SPHERE, SolidPrimitive.CYLINDER):
        r = d[SolidPrimitive.SPHERE_RADIUS] if prim.type == SolidPrimitive.SPHERE else d[SolidPrimitive.CYLINDER_RADIUS]
        h = 2 * r if prim.type == SolidPrimitive.SPHERE else d[SolidPrimitive.CYLINDER_HEIGHT]
        na, nz = max(8, int(math.ceil(2 * math.pi * r / step))), max(2, int(math.ceil(h / step)) + 1)
        a = np.linspace(0, 2 * math.pi, na, endpoint=False)
        z = np.linspace(-h / 2, h / 2, nz)
        if prim.type == SolidPrimitive.SPHERE:                       # 按纬度缩半径
            rr = np.sqrt(np.maximum(r * r - z * z, 0.0))
        else:
            rr = np.full(nz, r)
        side = np.stack([np.outer(rr, np.cos(a)).ravel(), np.outer(rr, np.sin(a)).ravel(), np.repeat(z, na)], 1)
        if prim.type == SolidPrimitive.SPHERE:
            return side
        g = np.arange(-r, r + step, step)                            # 圆柱两端盖
        X, Y = np.meshgrid(g, g)
        m = X ** 2 + Y ** 2 <= r * r
        caps = [np.stack([X[m], Y[m], np.full(m.sum(), zz)], 1) for zz in (-h / 2, h / 2)]
        return np.concatenate([side] + caps)
    return np.empty((0, 3))


def _pose_mat(pose):
    o = pose.orientation
    M = np.eye(4)
    M[:3, :3] = quat_to_mat(o.x, o.y, o.z, o.w)
    M[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return M


def object_points(objects, step=OBJ_SAMPLE):
    """planning scene 里碰撞体(CollisionObject：mesh / box / sphere / cylinder)的表面点，在各自 header 系。"""
    out = []
    for co in objects:
        M0 = _pose_mat(co.pose)
        parts = [(_triangle_points(np.array([[[v.x, v.y, v.z] for v in (m.vertices[i] for i in t.vertex_indices)]
                                              for t in m.triangles]), step), mp) for m, mp in zip(co.meshes, co.mesh_poses)]
        parts += [(_primitive_points(pr, step), pp) for pr, pp in zip(co.primitives, co.primitive_poses)]
        for P, pose in parts:
            if len(P):
                M = M0 @ _pose_mat(pose)
                out.append(P @ M[:3, :3].T + M[:3, 3])
    return np.concatenate(out) if out else np.empty((0, 3))


def _duration_of(traj):
    t = traj.points[-1].time_from_start
    return t.sec + t.nanosec * 1e-9


def _smooth_pinned(X, w):
    """沿第 0 维做三遍滑动平均（≈二次 B 样条核，窗口 w 个点，导数连续）；两端按"点对称"延拓，首尾值严格不变。"""
    if w <= 1 or len(X) < 3:
        return X
    h = w // 2
    for _ in range(3):
        k = min(h, len(X) - 1)
        pad = np.concatenate([2 * X[:1] - X[k:0:-1], X, 2 * X[-1:] - X[-2:-k - 2:-1]])
        c = np.cumsum(np.concatenate([np.zeros((1,) + X.shape[1:]), pad]), axis=0)
        X = (c[2 * k + 1:] - c[:-2 * k - 1]) / (2 * k + 1)
    return X


def cart_profile(kin, path, vel_limits, smooth=CART_SMOOTH):
    """笛卡尔路径的时间参数化（纯函数，planner_bench 也用它）。末端根本没动返回 None，否则 (t, s, v, Q)：
    沿末端弧长 s(平移 + LM_ANG_W·转角，每 CART_DS 一个点)的时间、弧长、路径速度 ṡ、关节角（相邻点间 s̈ 为常数）。
    关节角沿 s 先做 smooth(m) 窗口的平滑（首尾不动）：去掉 1cm IK 小步的抖动，绕障折线的拐角倒成小圆角(末端偏几 mm)
      ——不平滑的话拐角处关节速度是跳变的，JTC 样条会在那一下猛甩。平滑后的路径要另行复查碰撞（_cart_traj 做）。
    做法（路径速度 ṡ 的前向/后向扫描，即简化版 TOPP）：
      q̇ = q'ṡ，q̈ = q''ṡ² + q's̈。每点 ṡ 的上限取：末端平移 ≤ CART_SPEED、|q'_j|ṡ ≤ URDF 速度限位、
      |q''_j|ṡ² ≤ ½·CART_JOINT_ACCEL；s̈ 的上限取 CART_ACCEL 和 |q'_j|s̈ ≤ ½·CART_JOINT_ACCEL。
      两半加起来保证 |q̈_j| ≤ CART_JOINT_ACCEL（保守、不会无解）。起止 ṡ=0，正扫限加速、反扫限减速。"""
    P = np.asarray(path, dtype=float)
    fk = [kin.fk(q) for q in P]
    dp = np.array([np.linalg.norm(b[0] - a[0]) for a, b in zip(fk[:-1], fk[1:])])
    dr = np.array([np.linalg.norm(rotvec(b[1] @ a[1].T)) for a, b in zip(fk[:-1], fk[1:])])
    # 弧长参数 = 平移 + LM_ANG_W·转角：原地转姿态时也沿积分路径匀滑地走（而不是退化成两点）
    d = np.concatenate([[0.0], np.cumsum(dp + LM_ANG_W * dr)])
    d_tr = np.concatenate([[0.0], np.cumsum(dp)])
    D = float(d[-1])
    if D < 1e-6:
        return None
    keep = [0]                                                  # 去掉末端几乎没动的点(弧长要严格递增)
    for i in range(1, len(P)):
        if d[i] - d[keep[-1]] > 1e-6:
            keep.append(i)
    keep[-1] = len(P) - 1                                       # 终点一定保留
    P, d, d_tr = P[keep], d[keep], d_tr[keep]
    n = max(3, int(math.ceil(D / CART_DS)) + 1)
    s = np.linspace(0.0, D, n)
    ds = s[1]
    Q = np.stack([np.interp(s, d, P[:, j]) for j in range(P.shape[1])], 1)
    Q = _smooth_pinned(Q, max(1, int(round(smooth / ds))))
    q1 = np.gradient(Q, ds, axis=0)
    q1_hi = np.abs(q1)
    q2 = np.zeros_like(Q)                                        # 二阶差分（不用 gradient 套两次：会把尖峰抹平一半）
    q2[1:-1] = np.abs(Q[2:] - 2 * Q[1:-1] + Q[:-2]) / ds ** 2
    q2p = np.pad(q2, ((2, 2), (0, 0)), mode="edge")
    q2 = np.max([q2p[k:k + len(q2)] for k in range(5)], axis=0)  # 前后各取 2 点的最大值，保守
    r = np.clip(np.gradient(np.interp(s, d, d_tr), ds), 0.0, 1.0)   # 平移占弧长的比例
    a_j = 0.5 * CART_JOINT_ACCEL
    with np.errstate(divide="ignore"):
        x_max = np.minimum.reduce([
            (CART_SPEED / r) ** 2,
            np.min((np.asarray(vel_limits) / q1_hi) ** 2, axis=1),
            np.min(a_j / q2, axis=1)])                         # x = ṡ²
        u_max = np.minimum(CART_ACCEL, np.min(a_j / q1_hi, axis=1))
    u_seg = np.minimum(u_max[:-1], u_max[1:])
    x = x_max.copy()
    x[0] = x[-1] = 0.0
    for i in range(n - 1):                                      # 正扫：加速受限
        x[i + 1] = min(x[i + 1], x[i] + 2 * u_seg[i] * ds)
    for i in range(n - 2, -1, -1):                              # 反扫：减速受限
        x[i] = min(x[i], x[i + 1] + 2 * u_seg[i] * ds)
    v = np.sqrt(np.maximum(x, 0.0))
    t = np.concatenate([[0.0], np.cumsum(2 * ds / np.maximum(v[:-1] + v[1:], 1e-9))])
    return t, s, v, Q


def spline_peaks(tt, P, V):
    """按 JTC 的 位置+速度 分段三次 Hermite 插值，求各关节的速度峰值、加速度峰值（每段加速度线性，峰值在段端点）。"""
    h = np.diff(tt)[:, None]
    dP, v0, v1 = np.diff(P, axis=0), V[:-1], V[1:]
    acc = np.maximum(np.abs(6 * dP - (4 * v0 + 2 * v1) * h), np.abs(-6 * dP + (2 * v0 + 4 * v1) * h)) / h ** 2
    s = np.linspace(0, 1, 9)[:, None, None]                   # 速度段内是二次的：取几个点
    vel = np.abs(6 * (s - s ** 2) * dP / h + (1 - 4 * s + 3 * s ** 2) * v0 + (3 * s ** 2 - 2 * s) * v1)
    return vel.max(axis=(0, 1)), acc.max(axis=0)


def cart_samples(t, s, v, Q):
    """cart_profile 的结果按 CART_DT 取点（发给 JTC 的点：时间、关节角、关节速度；首尾速度 0）。
    相邻弧长点之间 s̈ 为常数，s(t) 按二次式插值（直接对 t 线性插值的话，起步那一段会变成瞬间跳到匀速）。
    最后按 JTC 实际插出来的样条复查关节加速度：离散误差偶尔超 CART_JOINT_ACCEL 几成 → 整条按比例拉长时间。"""
    Qs = CubicSpline(s, Q, axis=0)                             # 关节角对 s 三次插值（线性插值在低速、取点比 CART_DS 还密时会露出折角）
    for _ in range(4):
        tt = np.append(np.arange(0.0, t[-1], CART_DT), t[-1])
        if len(tt) > 2 and tt[-1] - tt[-2] < 0.5 * CART_DT:    # 最后一个间隔太短就并掉
            tt = np.delete(tt, -2)
        i = np.clip(np.searchsorted(t, tt, side="right") - 1, 0, len(t) - 2)
        tau = tt - t[i]
        acc = (v[i + 1] ** 2 - v[i] ** 2) / (2 * (s[i + 1] - s[i]))
        ss = np.clip(s[i] + v[i] * tau + 0.5 * acc * tau ** 2, s[i], s[i + 1])
        ss[-1] = s[-1]
        P = Qs(ss)
        P[0], P[-1] = Q[0], Q[-1]
        # 速度取过这些点的夹持三次样条（首尾速度 0）在各点的导数：JTC 按 位置+速度 做分段三次插值，
        #   这样它插出来的就是这条 C2 样条，加速度在点与点之间连续（用中心差分只到 C1，点上加速度会跳）
        V = CubicSpline(tt, P, axis=0, bc_type="clamped")(tt, 1) if len(tt) > 2 else np.zeros_like(P)
        ratio = float(spline_peaks(tt, P, V)[1].max()) / CART_JOINT_ACCEL
        if ratio <= 1.0:
            break
        k = math.sqrt(ratio) * 1.02                            # 时间 ×k → 加速度 ÷k²
        t, v = t * k, v / k
    return tt, P, V


class Plan:
    """一段规划好的手臂运动：路点(按 joints 顺序，用于复核/摘要) + 已经时间参数化好的 JointTrajectory。"""
    def __init__(self, arm, joints, path, traj, note):
        self.arm, self.joints, self.path, self.traj, self.note = arm, joints, path, traj, note
        self.duration = _duration_of(traj)

    def summary(self):
        P = np.asarray(self.path)
        steps = np.abs(np.diff(P, axis=0)) if len(P) > 1 else np.zeros((1, P.shape[1]))
        return (f"{self.note}：{len(self.traj.points)} 个轨迹点，{self.duration:.1f}s，"
                f"单步最大关节变化 {np.degrees(steps.max()):.2f}°，"
                f"关节总行程 {np.degrees(steps.sum()):.1f}°")


class ArmMotion(Node):
    def __init__(self, node_name="g2_arm_motion"):
        super().__init__(node_name)
        self.urdf = None
        self.kins, self.solvers, self.bkin, self.vel_limits = {}, {}, None, {}
        self.cur, self.cur_t = {}, {}
        self._lock = threading.Lock()
        self._warned_sv = False
        self._active_goal = None                               # 正在执行的 JTC 目标(cancel() 用)

        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, "/robot_description", self._on_urdf, qos_latched)
        # BEST_EFFORT：GDK FastDDS 配置(env_g2.sh)让 joint_state_broadcaster 变成 BEST_EFFORT 发布，RELIABLE 订阅收不到
        self.create_subscription(JointState, "/joint_states", self._on_js, qos_profile_sensor_data)
        self.actions = {a: ActionClient(self, FollowJointTrajectory, ACTION[a]) for a in ARMS}
        self._sv = self.create_client(GetStateValidity, "/check_state_validity")
        self._scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")

    # ── 输入 ──────────────────────────────────────────────────────────
    def _on_urdf(self, msg):
        if self.urdf is not None:
            return
        # 先在局部变量里把模型全建好，最后才设 self.urdf：ready() 在别的线程里以 self.urdf 非空为"模型已就绪"，
        #   先设 urdf 再建模型会让它读到半成品（2026-11-02 pose_ab_search 启动 KeyError: 'left'）
        urdf = msg.data
        robot = URDF.from_xml_string(urdf)
        kins = {a: arm_kin(urdf, a) for a in ARMS}
        self.solvers = {a: ArmSolver(kins[a]) for a in ARMS}
        self.vel_limits = {a: np.array([robot.joint_map[j].limit.velocity for j in kins[a].joint_names], dtype=float)
                           for a in ARMS}
        self.bkin = body_kin(urdf)
        self.kins = kins
        self.urdf = urdf
        self.get_logger().info("URDF 已加载：手臂链 arm_base_link→arm_{l,r}_end_link，腰链 base_link→arm_base_link")

    def _on_js(self, msg):
        now = time.monotonic()
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                self.cur[n] = p
                self.cur_t[n] = now

    def ready(self):
        return self.urdf is not None and all(j in self.cur for a in ARMS for j in self.kins[a].joint_names)

    def _fresh(self, joints):
        """取这组关节的最新值；任一缺失或超过 JOINT_MAX_AGE 就抛 RuntimeError。"""
        now = time.monotonic()
        with self._lock:
            missing = [j for j in joints if j not in self.cur]
            stale = [j for j in joints if j in self.cur_t and now - self.cur_t[j] > JOINT_MAX_AGE]
            if missing or stale:
                raise RuntimeError(f"关节反馈不可用：缺 {missing} / 过期 {stale}（hardware.launch 在跑吗？）")
            return np.array([self.cur[j] for j in joints])

    def q_arm(self, arm):
        return self._fresh(self.kins[arm].joint_names)

    # ── base_link ↔ 手臂根系(arm_base_link) ─────────────────────────────
    def _root_in_base(self):
        """实测腰关节 → arm_base_link 在 base_link 系的 (p, R)。"""
        return self.bkin.fk(self._fresh(self.bkin.joint_names))

    def to_root(self, p_base, R_base):
        pb, Rb = self._root_in_base()
        return Rb.T @ (np.asarray(p_base) - pb), (None if R_base is None else Rb.T @ R_base)

    def pose(self, arm):
        """当前末端位姿，base_link 系 (p, R)。"""
        pb, Rb = self._root_in_base()
        p, R = self.kins[arm].fk(self.q_arm(arm))
        return pb + Rb @ p, Rb @ R

    # ── 碰撞检查 / 绕行（MoveIt 服务） ──────────────────────────────────
    def _full_state(self, joints, q_arm):
        with self._lock:
            merged = dict(self.cur)
        for j, v in zip(joints, q_arm):
            merged[j] = float(v)
        js = JointState()
        js.name = list(merged.keys())
        js.position = [merged[k] for k in js.name]
        return js

    @staticmethod
    def _wait_future(fut, timeout):
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.002)
        return fut.result() if fut.done() else None

    def valid(self, arm, q):
        """整机自碰撞校验：当前全身 + 这条手臂设成 q，问 MoveIt 整台机器人撞不撞（含另一条臂/腰/头/底盘，
        group 留空 = 整机，同 genie pose_ab）。服务不可用时降级放行(True)并只警告一次；超时保守当撞(False)。"""
        if not self._sv.service_is_ready():
            if not self._warned_sv:
                self.get_logger().warn("/check_state_validity 不可用(move_group 没起?)，本次跳过碰撞检查")
                self._warned_sv = True
            return True
        req = GetStateValidity.Request()
        req.robot_state.joint_state = self._full_state(self.kins[arm].joint_names, q)
        # is_diff=True：在 move_group 当前状态上只改关节，保留附着物（attach_board.py 挂的纸板）。
        #   False 会把附着物全部清掉——2026-11-02 实测挂着纸板插进料框的姿态照样报"不撞"
        req.robot_state.is_diff = True
        res = self._wait_future(self._sv.call_async(req), SV_TIMEOUT)
        return bool(res.valid) if res is not None else False

    def path_clear(self, arm, path):
        """整条路径碰撞校验：关节每变化 CHECK_STEP(2°) 采一次（相邻路点之间也插值采），终点必查。"""
        if not self._sv.service_is_ready():
            self.valid(arm, path[-1])          # 触发一次"不可用"警告
            return True
        P = np.asarray(path)
        if not self.valid(arm, P[0]):
            return False
        for q0, q1 in zip(P[:-1], P[1:]):
            n = max(1, math.ceil(float(np.max(np.abs(q1 - q0))) / CHECK_STEP))
            for s in np.linspace(0.0, 1.0, n + 1)[1:]:
                if not self.valid(arm, q0 + s * (q1 - q0)):
                    return False
        return True

    def obstacles_root(self, arm):
        """末端粗筛用的障碍点，换到手臂链根系(arm_base_link)：MoveIt 当前 octomap 的占据体素中心 + planning scene 里
        碰撞体(add_collision_mesh.py 加的虚拟障碍等)的表面采样点。取不到返回空数组。
        碰撞体相机看不到、不在 octomap 里：以前粗筛对它是瞎的，每个穿过它的绕行模板都要真走 IK + 整机碰撞(~50ms/段)才被拒，
        2026-11-02 料框测试 8s 预算耗光（模板 9.4s 才找到）；加上后同一题 0.6s。"""
        if not self._scene_cli.service_is_ready():
            self.get_logger().warn("/get_planning_scene 不可用，网格搜索不做障碍粗筛（碰撞仍由 MoveIt 整机检查）")
            return np.empty((0, 3))
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                     PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        res = self._wait_future(self._scene_cli.call_async(req), 3.0)
        if res is None:
            return np.empty((0, 3))
        world = res.scene.world
        parts = []
        om = world.octomap
        if om.header.frame_id not in ("", "base_link"):
            self.get_logger().warn(f"octomap 在 {om.header.frame_id} 系，不是 base_link，不做 octomap 粗筛")
        else:
            parts.append(occupied_centers(om.octomap))
        objs = [co for co in world.collision_objects if co.header.frame_id in ("", "base_link")]
        if len(objs) < len(world.collision_objects):
            self.get_logger().warn("有碰撞体不在 base_link 系，粗筛不算它（碰撞仍由 MoveIt 整机检查）")
        parts.append(object_points(objs))
        P = np.concatenate([q for q in parts if len(q)]) if any(len(q) for q in parts) else np.empty((0, 3))
        if len(P) == 0:
            return P
        pb, Rb = self._root_in_base()
        return (P - pb) @ Rb                                   # base_link → arm_base_link: Rbᵀ(p − pb)

    # ── 时间参数化 ────────────────────────────────────────────────────
    def _cart_traj(self, arm, path):
        """笛卡尔路径(很多 1cm 小步)：cart_profile 沿路径变速（末端匀速 CART_SPEED，单关节超速/超加速度的地方就地压速），
        按 CART_DT 取点发给 JTC，路点带速度、中途不停。"""
        kin = self.kins[arm]
        prof = cart_profile(kin, path, self.vel_limits[arm])
        if prof is None:                                            # 末端根本没动
            P = np.asarray(path, dtype=float)
            return make_trajectory(np.array([P[0], P[-1]]), kin.joint_names,
                                   self.vel_limits[arm], JOINT_SETTINGS)[0]
        tt, P, V = cart_samples(*prof)
        if CART_SMOOTH > 0 and not self.path_clear(arm, P):     # 倒圆角后的路径会撞 → 退回不平滑（拐角处会慢下来）
            self.get_logger().warn("平滑后的笛卡尔路径会撞，退回原路径")
            tt, P, V = cart_samples(*cart_profile(kin, path, self.vel_limits[arm], smooth=0.0))
        traj = JointTrajectory(joint_names=list(kin.joint_names))
        for q, v, ti in zip(P, V, tt):
            pt = JointTrajectoryPoint(positions=[float(x) for x in q], velocities=[float(x) for x in v])
            pt.time_from_start.sec, pt.time_from_start.nanosec = divmod(round(ti * 1e9), 10**9)
            traj.points.append(pt)
        return traj

    # ── 规划 ──────────────────────────────────────────────────────────
    def plan_cartesian(self, arm, p_base, R_base=None, mode="full"):
        """积分笛卡尔直线到 base_link 系目标。R_base=None 时保持当前姿态。返回 Plan 或失败原因(str)。"""
        kin, S = self.kins[arm], self.solvers[arm]
        q0 = self.q_arm(arm)
        if R_base is None:
            R_base = self.pose(arm)[1]
        p_t, R_t = self.to_root(p_base, R_base)
        p0, R0 = kin.fk(q0)
        dist = float(np.linalg.norm(p_t - p0))
        # path_sane 的总行程上限按"末端走了多远"算；sanofi 原版只算平移，原地转姿态时上限被压到 0.3rad、
        #   正常转动也过不了。这里把转角按 LM_ANG_W(0.2 m/rad，IK 里统一位置/姿态量纲的同一个系数)折成距离。
        turn = float(np.linalg.norm(rotvec(R_t @ R0.T))) if mode == "full" else 0.0
        path, ok, ep, er, q_end = S.solve(q0, p_t, mode, R_t, R_t[:, 0])
        if not ok:
            return f"笛卡尔积分没到位(位置差 {ep*1000:.1f}mm，姿态差 {np.degrees(er):.1f}°)——超出工作空间或撞限位"
        if mode == "full":                                         # 终点 3mm → 0.1mm
            q_fine = polish(kin, q_end, p_t, R_t, POLISH_ITERS)
            if q_fine is not None and float(np.max(np.abs(q_fine - q_end))) <= STEP_JOINT_MAX:
                path = path + [q_fine]
        bad = path_sane(path, dist + LM_ANG_W * turn, "笛卡尔路径")
        if bad:
            return bad
        if not self.path_clear(arm, path):
            return "笛卡尔路径会撞"
        return Plan(arm, kin.joint_names, path, self._cart_traj(arm, path),
                    f"积分笛卡尔直线 {dist*100:.1f}cm" + (f" 转 {np.degrees(turn):.0f}°" if turn > 1e-3 else ""))

    def _solve_goal(self, arm, p_base, R_base, mode):
        """多种子 IK(终点整机不撞) + LM 精修。返回 (q0, q_goal 或 None, 失败原因, 耗时s)。"""
        kin, S = self.kins[arm], self.solvers[arm]
        q0 = self.q_arm(arm)
        if R_base is None:
            R_base = self.pose(arm)[1]
        p_t, R_t = self.to_root(p_base, R_base)
        t0 = time.monotonic()
        q = S.ik(p_t, mode, R_t, R_t[:, 0], q0, valid=lambda q: self.valid(arm, q))
        if q is None:
            q_any = S.ik(p_t, mode, R_t, R_t[:, 0], q0)
            reason = "IK 无解(目标够不着或姿态做不到)" if q_any is None else "有 IK 解，但都会撞"
            return q0, None, reason, time.monotonic() - t0
        if mode == "full":
            q_fine = polish(kin, q, p_t, R_t, POLISH_ITERS)
            if q_fine is not None and self.valid(arm, q_fine):
                q = q_fine
        return q0, q, "", time.monotonic() - t0

    def plan_pose(self, arm, p_base, R_base=None, mode="full"):
        """IK 解终点 → 两点五次多项式直连；直连会撞就末端空间网格搜索绕行（不用 OMPL）。
        返回 Plan 或失败原因(str)。"""
        kin = self.kins[arm]
        q0, q_goal, reason, t_ik = self._solve_goal(arm, p_base, R_base, mode)
        if q_goal is None:
            return reason
        path = interp(q0, q_goal)
        if self.path_clear(arm, path):
            traj = make_trajectory(np.array([q0, q_goal]), kin.joint_names,
                                   self.vel_limits[arm], JOINT_SETTINGS)[0]
            return Plan(arm, kin.joint_names, path, traj, f"IK({t_ik*1000:.0f}ms) + 五次多项式直连")
        plan = self.plan_detour(arm, p_base, R_base)
        if isinstance(plan, str):
            return f"关节直连会撞；{plan}"
        plan.note = f"关节直连会撞 → {plan.note}"
        return plan

    def plan_detour(self, arm, p_base, R_base=None, planner=None):
        """末端绕障规划（planners/ 里的算法，默认 planners.DEFAULT；离线对比见 planner_bench）：先试直线，
        不行再绕。每段积分笛卡尔 + 整机碰撞检查，全程构型连续，不会抡大圈。返回 Plan 或原因(str)。"""
        kin, S = self.kins[arm], self.solvers[arm]
        algos, _ = planners.discover()
        name = planner or planners.DEFAULT
        if name not in algos:
            return f"没有规划器 {name}（可选：{', '.join(algos)}）"
        q0 = self.q_arm(arm)
        if R_base is None:
            R_base = self.pose(arm)[1]
        p_t, R_t = self.to_root(p_base, R_base)
        obstacles = self.obstacles_root(arm)
        up = self.to_root(np.zeros(3), np.eye(3))[1] @ np.array([0.0, 0.0, 1.0])   # base_link +z 在根系里
        req = planners.PlanRequest(solver=S, valid=lambda q: self.valid(arm, q), q0=q0, p_goal=p_t, R_goal=R_t,
                                   obstacles=obstacles, up=up)
        res = algos[name].factory().plan(req)
        if not res.ok:
            return f"[{name}] {res.note}"
        jp = res.joint_path
        q_fine = polish(kin, jp[-1], p_t, R_t, POLISH_ITERS)          # 终点 3mm → 0.1mm
        if q_fine is not None and float(np.max(np.abs(q_fine - jp[-1]))) <= STEP_JOINT_MAX and self.valid(arm, q_fine):
            jp = list(jp) + [q_fine]
        return Plan(arm, kin.joint_names, jp, self._cart_traj(arm, jp), f"[{name}] {res.note}；粗筛障碍点(octomap+碰撞体) {len(obstacles)}")

    def plan_ik_only(self, arm, p_base, R_base=None, mode="full"):
        """只解 IK 不出路径(看解和关节差用)。返回 (q_goal 或 None, 耗时 s)。"""
        _, q, _, dt = self._solve_goal(arm, p_base, R_base, mode)
        return q, dt

    # ── 执行 ──────────────────────────────────────────────────────────
    def execute(self, plan, wait=True):
        """把 Plan 的轨迹通过 FollowJointTrajectory action 发给该臂 JTC。
        wait=True：阻塞到 JTC 返回结果，成功后再按关节反馈确认停稳；期间 Ctrl-C 会取消目标。
        返回 (成功?, 说明)。"""
        client = self.actions[plan.arm]
        if not client.wait_for_server(timeout_sec=ACTION_WAIT):
            return False, f"{ACTION[plan.arm]} 不在线(hardware.launch 没跑？)，不发"
        q_now = self.q_arm(plan.arm)
        drift = float(np.max(np.abs(q_now - np.asarray(plan.path[0]))))
        if drift > 0.05:
            return False, f"手臂已离开规划起点 {np.degrees(drift):.1f}°，请重新规划"
        handle = self._wait_future(client.send_goal_async(FollowJointTrajectory.Goal(trajectory=plan.traj)),
                                   ACTION_WAIT)
        if handle is None or not handle.accepted:
            return False, "JTC 拒绝了目标"
        self._active_goal = handle
        if not wait:
            return True, "已发出"
        result_future = handle.get_result_async()
        t0 = time.monotonic()
        try:
            while not result_future.done():
                if time.monotonic() - t0 > plan.duration + RESULT_MARGIN:
                    handle.cancel_goal_async()
                    return False, f"超过 {plan.duration + RESULT_MARGIN:.1f}s 没拿到执行结果，已取消"
                time.sleep(0.01)
        except KeyboardInterrupt:
            handle.cancel_goal_async()
            return False, "已取消（Ctrl-C）"
        res = result_future.result()
        self._active_goal = None
        if res.status == 5:                                        # 5 = CANCELED
            return False, "已取消"
        if res.status != 4 or res.result.error_code != 0:          # 4 = SUCCEEDED
            return False, f"JTC 执行失败 status={res.status} error_code={res.result.error_code} {res.result.error_string}"
        ok, msg = self.wait_settled(plan.arm, np.asarray(plan.path[-1]), SETTLE_TIMEOUT)
        return ok, f"JTC 成功（{time.monotonic() - t0:.1f}s）；{msg}"

    def cancel(self):
        """取消正在执行的 JTC 目标（别的线程里 execute() 会收到 CANCELED 返回）。没有在执行就返回 False。"""
        h = self._active_goal
        if h is None:
            return False
        h.cancel_goal_async()
        return True

    def wait_settled(self, arm, q_goal, timeout):
        """等手臂到终点附近(每关节 ≤ SETTLE_TOL)且连续 SETTLE_STILL 秒不动。"""
        t0 = time.monotonic()
        still_since, last = None, None
        while time.monotonic() - t0 < timeout:
            try:
                q = self.q_arm(arm)
            except RuntimeError:
                time.sleep(0.02)
                continue
            near = float(np.max(np.abs(q - q_goal))) <= SETTLE_TOL
            moving = last is not None and float(np.max(np.abs(q - last))) > 1e-3
            last = q
            if near and not moving:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since >= SETTLE_STILL:
                    return True, "实测已停稳在终点"
            else:
                still_since = None
            time.sleep(0.05)
        err = float(np.max(np.abs(self.q_arm(arm) - q_goal)))
        return False, f"{timeout:.1f}s 内实测未停稳在终点，最大关节差 {np.degrees(err):.2f}°"

    # ── 便捷接口 ──────────────────────────────────────────────────────
    def cartesian_to(self, arm, p_base, R_base=None, mode="full"):
        plan = self.plan_cartesian(arm, p_base, R_base, mode)
        return (False, plan) if isinstance(plan, str) else self.execute(plan)

    def cartesian_rel(self, arm, dp_base, drot_base=None):
        """相对当前末端：base_link 系平移 dp(m)、绕 base_link 轴转 drot(旋转矩阵，可 None)，走直线。"""
        p, R = self.pose(arm)
        R_t = R if drot_base is None else drot_base @ R
        return self.cartesian_to(arm, p + np.asarray(dp_base), R_t)

    def pose_to(self, arm, p_base, R_base=None, mode="full"):
        plan = self.plan_pose(arm, p_base, R_base, mode)
        return (False, plan) if isinstance(plan, str) else self.execute(plan)
