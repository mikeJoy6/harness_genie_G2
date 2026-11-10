#!/usr/bin/env python3
"""algorithms.py —— G2 手臂运控的两个核心算法（纯计算，不碰 ROS，可离线单测）。

移植自 sanofi_trash_grasping/src/trash_perception/trash_perception/grasp_server.py（该项目里最成熟的一版），
从 GraspServer 节点里拆成与机器人/ROS 无关的 ArmSolver；常量、步骤、判据保持原样。

一、积分笛卡尔 solve()（源头：tiangong2_wbc_ws/src/tiangong2_wbc/scripts/differential_ik.py）
    从【实测当前构型】出发，雅可比 DLS 逐小步(≤1cm / ≤0.03rad)把末端拉到目标，每步重算 FK 闭环。
    每步位移都很小 → 相邻构型连续 → 结构上不会换分支、不会"末端走一点、胳膊抡一大圈"。
    limit_step()：某关节撞限位还往外推就把它锁住，剩下的关节接管(否则会原地卡死)。
    path_sane()：事后兜底，单步跳变 / 总行程 / 终点换分支 三条任一超标整条作废。

二、多种子自适应 LM IK ik()（源头：g1_pointcloud_grasping/src/genie_motion/genie_motion/algorithms/_adaptive_ik.py）
    q0 + IK_SEEDS 个随机种子，每个种子 lm() 最多 LM_ITERS(=100) 轮：
      误差 e=[位置; 0.2·姿态]、信赖域限步长、按【真实 FK 误差】回退验收、
      实际/预测下降比自适应调阻尼和半径、限位上往外推的关节冻结、连续 6 次不降就放弃。
    所有合格解按离 q0 排序，最近 PULL_TOP 个再 pull_near() 沿零空间往 q0 滑 → 关节插值过去不抡大圈。
    给了 valid(q) 回调(MoveIt 碰撞检查)时，取最近且不撞的那个。

姿态模式 mode：
    full  追整姿态 R_tgt（6 维全约束；终点 IK 用 KDL LMA，C++ 快）
    axis  只让末端 X 轴对准 x_t（绕 X 滚转自由，5 维）
    pos   只管位置（够不够得着的分诊用）
（sanofi 还有个 band 模式 = axis + "手心朝下 45°~80°"，是抓垃圾袋专用的任务约束，没搬。）
"""
import numpy as np

from .kin import dls_pinv

# 积分笛卡尔的步长/容差
LIN_STEP, ROT_STEP = 0.01, 0.03      # 每小步最多走 1cm / 0.03rad
POS_TOL, ROT_TOL = 0.003, 0.02       # 收敛：位置 3mm，姿态 ~1.1°
LAM = 0.05                           # DLS 阻尼
MAX_ITERS = 4000
# 约束 C：防大回环（sanofi 实测：正常路径每步末端≤1cm 时单步关节最大 0.079；累计转角/直线距离 中位 12、95 分位 30）
STEP_JOINT_MAX = 0.1                 # 笛卡尔路径相邻两点任一关节变化上限(rad)
TRAVEL_RATIO = 30.0                  # 关节累计转角和 ≤ 它 × 末端直线距离(rad/m)
BRANCH_TOL = 0.1                     # 终点与给定 IK 解的关节差上限(rad)，防换分支
# 终点 IK 用自适应 LM：只求终点不出路径，所以不必小步积分；失败种子几步内就放弃(原来要跑满 1200 步)
IK_SEEDS = 60                        # q0 之外的随机重启种子数
LM_ITERS = 100                       # 单个种子最多迭代
LM_ANG_W = 0.2                       # 姿态误差权重(m/rad)：统一位置/姿态量纲
LM_DAMP0, LM_RADIUS0 = 0.01, 0.25    # 初始阻尼 / 单步关节最大改变量(rad)
LM_STALL = 6                         # 连续这么多次回退都不降误差 → 判停滞放弃
# 解拉近：7 自由度臂同一个末端目标有一整条解(零空间自运动)。沿零空间把它往 q0 滑，
#   每步再用 LM 拉回约束面、真变近才收 → 终点离当前构型最近，关节插值过去不抡大圈
PULL_TOP = 8                         # 只拉近离 q0 最近的这么多个候选(再重新排序)
PULL_STEP = 0.2                      # 零空间单步最大关节改变量(rad)，不收就减半
PULL_ITERS = 40                      # 最多拉几步
PULL_FIX_ITERS = 15                  # 每步拉回约束面的 LM 迭代上限


def rotvec(R):
    """旋转矩阵 → 旋转向量(axis*angle)：把姿态误差变成 3 维角速度误差。"""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = float(np.arccos(c))
    if ang < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    nw = np.linalg.norm(w)
    if nw < 1e-9:                      # ~180°：从对角线抽轴
        axis = np.sqrt(np.clip((np.diag(R) + 1.0) / 2.0, 0.0, 1.0))
        return axis * ang
    return w / nw * ang


def rpy_to_mat(roll, pitch, yaw):
    """RPY(固定轴 X→Y→Z，即 URDF/ROS 约定) → 旋转矩阵。"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def mat_to_rpy(R):
    """旋转矩阵 → RPY(固定轴 X→Y→Z)。"""
    pitch = float(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
    if abs(np.cos(pitch)) < 1e-6:                       # 万向锁：yaw 并进 roll
        return float(np.arctan2(-R[1, 2], R[1, 1])), pitch, 0.0
    return float(np.arctan2(R[2, 1], R[2, 2])), pitch, float(np.arctan2(R[1, 0], R[0, 0]))


def limit_step(q, J, e, lower, upper):
    """限位感知的一步 IK（clamping 法）：dq=J⁺e 后，若某关节撞限位且还在往外推，
    就把它这一步【锁死】(从雅可比剔掉)，让剩下的关节重解、接管它的活 —— 迭代到没有
    新的越界为止。治的就是"某关节到头后其余关节不接管、原地卡死"。"""
    n = len(q)
    free = np.ones(n, dtype=bool)
    dq = np.zeros(n)
    for _ in range(n):                       # 最多锁 n 次（每次至少锁 1 个）
        dq = np.zeros(n)
        dq[free] = dls_pinv(J[:, free], LAM) @ e
        qn = q + dq
        over = free & (((qn < lower) & (dq < 0)) | ((qn > upper) & (dq > 0)))
        if not over.any() or not (free & ~over).any():
            break
        free[over] = False                   # 锁死这些越界关节，用剩下的重解
    return np.clip(q + dq, lower, upper)


def path_sane(path, cart_dist, label, q_end=None):
    """约束 C：笛卡尔路径不许出现大回环(末端走一点点、关节却抡一大圈)。
    单步任一关节变化 ≤ STEP_JOINT_MAX；各关节累计转角和 ≤ TRAVEL_RATIO×末端直线距离；
    给了 q_end 时终点须与它同分支(任一关节差 ≤ BRANCH_TOL)。通过返回空串，否则返回原因。"""
    P = np.asarray(path)
    steps = np.abs(np.diff(P, axis=0)) if len(P) > 1 else np.zeros((1, P.shape[1]))
    jump = float(steps.max())
    if jump > STEP_JOINT_MAX:
        return f"{label}单步关节跳变 {jump:.3f}rad(>{STEP_JOINT_MAX})"
    travel, limit = float(steps.sum()), TRAVEL_RATIO * max(cart_dist, 0.01)
    if travel > limit:
        return f"{label}关节总行程 {travel:.2f}rad 超过 {limit:.2f}rad(末端只走 {cart_dist*100:.0f}cm)"
    if q_end is not None:
        diff = float(np.max(np.abs(P[-1] - q_end)))
        if diff > BRANCH_TOL:
            return f"{label}终点与 IK 解不在同一分支(关节差 {diff:.3f}rad>{BRANCH_TOL})"
    return ""


def interp(q0, q1, max_step=0.02):
    """关节空间线性插值 q0→q1：单个 IK 解之间平滑过渡，无构型分支跳变（不大回环）。"""
    d = np.asarray(q1) - np.asarray(q0)
    steps = max(2, int(np.ceil(float(np.max(np.abs(d))) / max_step)))
    return [q0 + d * t for t in np.linspace(0.0, 1.0, steps)]


class ArmSolver:
    """一条手臂链上的 积分笛卡尔 + 多种子 IK。kin 是 kin.ChainKin（全部在链根系里算）。"""

    def __init__(self, kin, seed=0):
        self.kin = kin
        self._rng = np.random.default_rng(seed)   # IK 随机重启用（固定种子 → 结果可复现）

    # ── 姿态误差（solve 和 lm 共用） ──────────────────────────────────
    @staticmethod
    def rot_err(R, mode, R_tgt, x_t):
        """当前末端朝向 R → 姿态误差(旋转向量，链根系)。
             full 追整姿态 R_tgt；axis 只让末端 X 对准 x_t；pos 零"""
        if mode == "full":
            return rotvec(R_tgt @ R.T)
        if mode == "pos":
            return np.zeros(3)
        xc = R[:, 0]                                          # axis：把当前 X 轴转向 x_t，滚转不管
        ax = np.cross(xc, x_t)
        s, c = float(np.linalg.norm(ax)), float(np.dot(xc, x_t))
        return ax / s * float(np.arctan2(s, c)) if s > 1e-9 else np.zeros(3)

    def accept(self, q, p_tgt, mode, R_tgt, x_t):
        """收敛解的复核：在限位内 + 位置到位 + 姿态满足(按 mode)。不合格返回 False。"""
        kin = self.kin
        if q is None or np.any(q < kin.lower - 1e-6) or np.any(q > kin.upper + 1e-6):
            return False
        pos, R = kin.fk(q)
        if np.linalg.norm(pos - p_tgt) > POS_TOL:
            return False
        if mode == "full":
            return np.linalg.norm(rotvec(R_tgt @ R.T)) <= ROT_TOL
        if mode == "axis":
            return float(np.dot(R[:, 0], x_t)) >= np.cos(ROT_TOL)
        return True                                            # pos

    # ── 算法一：积分笛卡尔 ────────────────────────────────────────────
    def solve(self, q0, p_tgt, mode="full", R_tgt=None, x_t=None, max_iters=MAX_ITERS):
        """雅可比 DLS 积分：逐小步把末端拉到目标，返回 (关节路径, 是否到位, 位置差, 姿态差, 终点q)。"""
        kin = self.kin
        q = np.array(q0, dtype=float)
        path = [q.copy()]
        ep = er = 0.0
        for _ in range(max_iters):
            pos, R = kin.fk(q)
            dp = p_tgt - pos
            drot = self.rot_err(R, mode, R_tgt, x_t)
            ep, er = float(np.linalg.norm(dp)), float(np.linalg.norm(drot))
            if ep < POS_TOL and er < ROT_TOL:
                return path, True, ep, er, q
            if ep > LIN_STEP:
                dp = dp * (LIN_STEP / ep)
            if er > ROT_STEP:
                drot = drot * (ROT_STEP / er)
            if mode == "pos":
                J, e = kin.jacobian(q)[:3, :], dp
            else:
                J, e = kin.jacobian(q), np.concatenate([dp, drot])
            q = limit_step(q, J, e, kin.lower, kin.upper)
            path.append(q.copy())
        return path, False, ep, er, q

    # ── 算法二：多种子自适应 LM IK ────────────────────────────────────
    def seeds(self, q0, n):
        lo = np.where(np.isfinite(self.kin.lower), self.kin.lower, -np.pi)
        hi = np.where(np.isfinite(self.kin.upper), self.kin.upper, np.pi)
        return [np.asarray(q0, dtype=float)] + [self._rng.uniform(lo, hi) for _ in range(n)]

    def ik(self, p_tgt, mode, R_tgt, x_t, q0, n=IK_SEEDS, valid=None):
        """带随机重启的 IK：q0 + n 个随机种子都解，收集所有合格解，按离 q0 由近到远排。
        · valid=None：返回最近解。
        · valid(q)->bool 给定(MoveIt 碰撞检查)：返回【最近且 valid】的解 → 天然绕开身体；有解但全不 valid 返回 None。
        full 用 KDL LMA(C++，快)。axis/pos(滚转自由，LMA 不支持)：
          q0 种子用小步积分 solve(连续跟踪当前分支)；随机种子用自适应 LM(lm)，失败几步就放弃。
          ★不在 q0 解到就提前返回★：滚转自由时解是一整片，q0 积分出的那个常常不是最近的。
        最近的 PULL_TOP 个候选再沿零空间往 q0 拉近(pull_near)后重新排序：防大回环。"""
        kin = self.kin
        q0 = np.asarray(q0, dtype=float)
        sols = []
        seeds = self.seeds(q0, n)                              # seeds[0] = q0
        if mode == "full":
            cands = (kin.ik(p_tgt, R_tgt, seed) for seed in seeds)
        else:
            _, ok, _, _, q_first = self.solve(q0, p_tgt, mode, R_tgt, x_t, max_iters=1200)
            cands = [q_first if ok else None]
            cands += [self.lm(seed, p_tgt, mode, R_tgt, x_t) for seed in seeds[1:]]
        for q in cands:
            if self.accept(q, p_tgt, mode, R_tgt, x_t):
                sols.append(q)
        if not sols:
            return None
        dist = lambda q: float(np.linalg.norm(q - q0))
        sols.sort(key=dist)
        sols = [self.pull_near(q, q0, p_tgt, mode, R_tgt, x_t)
                for q in sols[:PULL_TOP]] + sols[PULL_TOP:]
        sols.sort(key=dist)
        if valid is None:
            return sols[0]
        for q in sols:                                         # 取最近的 valid(不撞身体)的分支
            if valid(q):
                return q
        return None                                            # 有 IK 解但全部 invalid

    def pull_near(self, q, q_ref, p_tgt, mode, R_tgt, x_t):
        """把一个合格解沿任务零空间往 q_ref 滑，末端目标不变，关节离 q_ref 尽量近。
        任务雅可比：pos=位置 3 行；axis=位置 + 垂直末端 X 的 2 个转动(绕 X 滚转放开)；full=整 6 行。
        每步：零空间方向 N(q_ref−q) 走 ≤PULL_STEP → lm 拉回约束面并复核 →
        【过 accept 且确实变近】才收，否则步长减半。只会变近不会变远。"""
        kin = self.kin
        best = float(np.linalg.norm(q - q_ref))
        step = PULL_STEP
        for _ in range(PULL_ITERS):
            J = kin.jacobian(q)
            if mode == "pos":
                Jt = J[:3]
            elif mode == "full":
                Jt = J
            else:
                x = kin.fk(q)[1][:, 0]
                Jt = np.vstack([J[:3], (np.eye(3) - np.outer(x, x)) @ J[3:]])
            d = (np.eye(len(q)) - np.linalg.pinv(Jt, rcond=1e-4) @ Jt) @ (q_ref - q)
            nd = float(np.linalg.norm(d))
            if nd < 1e-3:
                break                                          # 已是本分支上离 q_ref 最近的点
            q_try = np.clip(q + d * min(1.0, step / nd), kin.lower, kin.upper)
            q_try = self.lm(q_try, p_tgt, mode, R_tgt, x_t, max_iters=PULL_FIX_ITERS)
            if q_try is not None and float(np.linalg.norm(q_try - q_ref)) < best - 1e-4:
                q, best = q_try, float(np.linalg.norm(q_try - q_ref))
            else:
                step *= 0.5
                if step < 0.01:
                    break
        return q

    def lm(self, seed, p_tgt, mode, R_tgt, x_t, max_iters=LM_ITERS):
        """自适应阻尼最小二乘(LM)求终点关节角，收敛且过 accept 返回 q，否则 None。
        误差 e=[位置; LM_ANG_W·姿态]；每步 dq 被限在信赖域半径内，按【真实 FK 误差】回退验收，
        实际下降/预测下降比大就放宽阻尼、扩大半径，小就收紧；限位上且梯度往外推的关节冻结。"""
        kin = self.kin
        q = np.clip(np.array(seed, dtype=float), kin.lower, kin.upper)
        damp, radius = LM_DAMP0, LM_RADIUS0
        rot_rows = mode != "pos"                               # pos 只看位置 3 行

        def _eval(q):
            pos, R = kin.fk(q)
            dp = p_tgt - pos
            drot = self.rot_err(R, mode, R_tgt, x_t)
            e = np.concatenate([dp, LM_ANG_W * drot]) if rot_rows else dp
            return e, 0.5 * float(e @ e), float(np.linalg.norm(dp)), float(np.linalg.norm(drot))

        e, cost, ep, er = _eval(q)
        stalls = 0
        for _ in range(max_iters):
            if ep < POS_TOL and er < ROT_TOL and self.accept(q, p_tgt, mode, R_tgt, x_t):
                return q
            J = kin.jacobian(q)
            J = np.vstack([J[:3], LM_ANG_W * J[3:]]) if rot_rows else J[:3]
            grad = J.T @ e
            frozen = (((q <= kin.lower + 1e-10) & (grad < 0)) |
                      ((q >= kin.upper - 1e-10) & (grad > 0)))
            free = np.flatnonzero(~frozen)
            if free.size == 0:
                return None
            Jf = J[:, free]
            dq = np.zeros(len(q))
            dq[free] = np.linalg.solve(Jf.T @ Jf + damp * damp * np.eye(free.size), grad[free])
            big = float(np.max(np.abs(dq)))
            if not np.isfinite(big):
                return None
            if big > radius:
                dq *= radius / big
            for k in range(8):                                 # 回退：按真实误差验收这一步
                q_try = np.clip(q + dq * 0.5 ** k, kin.lower, kin.upper)
                e_try, cost_try, ep_try, er_try = _eval(q_try)
                if cost_try < cost:
                    pred = cost - 0.5 * float(np.sum((e - J @ (q_try - q)) ** 2))
                    ratio = (cost - cost_try) / pred if pred > 1e-16 else 0.0
                    if ratio > 0.75:
                        damp, radius = max(1e-6, damp * 0.5), min(0.6, radius * 1.5)
                    elif ratio < 0.25:
                        damp, radius = min(10.0, damp * 2.0), max(1e-6, radius * 0.7)
                    q, e, cost, ep, er = q_try, e_try, cost_try, ep_try, er_try
                    stalls = 0
                    break
            else:                                              # 8 次回退都没降 → 收紧再试
                stalls += 1
                damp, radius = min(10.0, damp * 4.0), max(1e-6, radius * 0.5)
                if stalls >= LM_STALL:
                    return None
        if ep < POS_TOL and er < ROT_TOL and self.accept(q, p_tgt, mode, R_tgt, x_t):
            return q
        return None


def polish(kin, q, p_tgt, R_tgt, max_iters=30):
    """终点精修：从 q 出发跑 genie 的自适应 LM(full 6D)，把 3mm/0.02rad 的解收紧到 0.1mm/0.001rad。
    收敛返回新 q，否则 None（调用方保留原解）。"""
    from .ik_lab.algorithms._adaptive_ik import solve_adaptive
    import time
    res = solve_adaptive(kin, q, p_tgt, R_tgt, max_iters, time.monotonic() + 1.0)
    return res.q if res.converged else None
