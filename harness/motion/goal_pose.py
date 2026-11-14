"""goal_pose.py —— 手臂终点姿态怎么定。纯计算(PyKDL + numpy),不碰 ROS;全部在手臂链根系 arm_base_link 里算。

规划器(积分笛卡尔、via_then_rrt)都要一个完整的终点姿态 R。以前 R = 当前姿态(“只会平移”),目标往身体中线一靠就 IK 无解。
这里给两种别的来源,算出 R 之后照样交给原来的规划器:

  free_goal     放松 pitch / yaw:先“只管位置”找到够得着的工具轴方向,再锁 roll(不自转)重解;锁 roll 解不出才用只管位置的解。
  ComfortPose   舒适姿态,三角函数闭式推(思路来自 ~/genie_framework/DEVLOG.md 7721~7888 “二次推导”):
                肩 S、肘 E、腕心 W、末端点 T 共面。
                  ① 腕心离肩的距离 d 取两个临界的中值:肘弯 90°(√(L1²+L2²))和伸直(L1+L2)。
                  ② 末端连杆长 c(腕心→末端)固定,三角形 S-W-T 三边 b、c、d 已知 → 余弦定理得 T 处夹角 θ。
                  ③ 定基面:这个面绕 S→T 转 φ。φ=0 是“广义水平”(面和水平面的交线 ⊥ S→T),再列几个 φ 做候选。
                  ④ 肘在同一个面里、往外(左臂 +y、右臂 −y)/往下的一侧,由 S-E-W 三角形定出肘的目标点 E*。
                每个候选:工具轴 Z = W→T(pitch / yaw 由几何定);roll 锁住 = 从当前姿态最小转动过去、不绕工具轴自转
                (roll_locked,可再加 roll_deg)→ IK → 零空间把肘拉向 E*(末端不动)
                → 打分(各关节离限位中点越近越舒服,可选“工具朝下”加分),取分最低的。

G2 手臂(URDF 零位,根系):肩心 link2(z=0.244),肘 link4(z=0.575),腕心 link6/7(z=0.853),末端 arm_*_end_link(z=0.943)。
L1≈0.331、L2≈0.278、c=0.090;末端坐标系的 Z 轴 = 腕心→末端方向(工具轴)。
"""
import math

import numpy as np

from .algorithms import polish
from .kin import ChainKin, dls_pinv

ROOT = "arm_base_link"
SIDE = {"left": "l", "right": "r"}


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


# ---------------------------------------------------------------- roll 约束
def _axis_angle(axis, angle):
    """绕单位轴 axis 转 angle 弧度的旋转矩阵(罗德里格斯公式)。"""
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def rotation_from_to(a, b):
    """把单位向量 a 转到 b 的最小旋转(转轴 ⊥ a、b)。"""
    axis = np.cross(a, b)
    s, c = float(np.linalg.norm(axis)), float(np.dot(a, b))
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        helper = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
        return _axis_angle(_unit(np.cross(a, helper)), math.pi)      # 正好反向:绕任一 ⊥ 轴转 180°
    return _axis_angle(axis / s, math.atan2(s, c))


def roll_locked(R_now, tool_axis, roll_deg=0.0):
    """终点姿态:工具轴(末端 Z)指向 tool_axis,绕工具轴**不自转**。

    做法:当前姿态 R_now 只做“把 Z 轴转到 tool_axis”的最小转动,X、Y 跟着带过去;
    roll_deg ≠ 0 时再绕新的工具轴转这么多(右手定则)。pitch / yaw 由 tool_axis 决定,roll 由这里决定。"""
    R = rotation_from_to(R_now[:, 2], _unit(np.asarray(tool_axis, float))) @ R_now
    if roll_deg:
        R = R @ _axis_angle(np.array([0.0, 0.0, 1.0]), math.radians(roll_deg))
    return R


def roll_change_deg(R_now, R_new):
    """R_new 比“不自转”多转了多少度(绕工具轴,带符号)。用来检查 / 报告。"""
    R_ref = roll_locked(R_now, R_new[:, 2])
    x_ref, x_new, z = R_ref[:, 0], R_new[:, 0], R_new[:, 2]
    return math.degrees(math.atan2(float(np.dot(np.cross(x_ref, x_new), z)), float(np.dot(x_ref, x_new))))


# ---------------------------------------------------------------- 放松
def free_goal(solver, q0, p_root, valid=None, roll_deg=0.0):
    """只放开 pitch / yaw:先“只管位置”找一个够得着的工具轴方向,再把 roll 锁住(不自转 + roll_deg)重解。
    返回 (q 或 None, 说明)。"""
    eye = np.eye(3)
    q_pos = None
    path, ok, _, _, q_end = solver.solve(q0, p_root, "pos", eye, eye[:, 0])
    if ok and (valid is None or valid(q_end)):
        q_pos = q_end
    else:
        q_pos = solver.ik(p_root, "pos", eye, eye[:, 0], q0, valid=valid)
    if q_pos is None:
        return None, "只管位置也够不着(或都会撞)"
    R_now = solver.kin.fk(q0)[1]
    R = roll_locked(R_now, solver.kin.fk(q_pos)[1][:, 2], roll_deg)
    q = solver.ik(p_root, "full", R, R[:, 0], q_pos, valid=valid)
    if q is not None:
        return q, ""
    return q_pos, f"(锁 roll 解不出,手腕绕工具轴转了 {roll_change_deg(R_now, solver.kin.fk(q_pos)[1]):.0f}°)"


# ---------------------------------------------------------------- 舒适姿态
class ComfortPose:
    PHI_DEG = (0, 30, 60, 90, 120, -30, -60)       # 基面绕 S→T 转多少度(0 = 广义水平),都试
    D_FRACTION = (0.5, 0.25, 0.75)                  # d 在 [肘 90°, 伸直] 之间取哪,0.5 = 中值

    def __init__(self, urdf_str, arm, solver):
        s = SIDE[arm]
        self.arm = arm
        self.solver = solver                        # 这条手臂的 ArmSolver(和规划器用同一个)
        self.kin = solver.kin                        # arm_base_link → arm_*_end_link
        self.elbow_kin = ChainKin(urdf_str, ROOT, f"arm_{s}_link4")
        P = self.kin.segment_positions(np.zeros(self.kin.n))
        self.S = P[1]                                # 肩心(link2 原点,只在 joint1 轴上,常量)
        elbow, wrist, end = P[3], P[5], P[-1]
        self.L1 = float(np.linalg.norm(elbow - self.S))
        self.L2 = float(np.linalg.norm(wrist - elbow))
        self.c = float(np.linalg.norm(end - wrist))
        self.mid = (self.kin.lower + self.kin.upper) / 2
        self.half = (self.kin.upper - self.kin.lower) / 2

    # ---- 几何 ----
    def candidates(self, T, up, outward, R_now=None, roll_deg=0.0):
        """T:末端目标点;up / outward:base_link 的“上”和“往外”在根系里的方向。
        R_now:当前末端姿态,给了就锁 roll(工具轴转过去、绕工具轴不自转,再加 roll_deg);
        不给就用基面法向当 Y(只在没有当前姿态时用)。
        返回候选列表,每个 {"phi_deg", "d", "W", "R", "E"};够不着返回 []。"""
        S, c, L1, L2 = self.S, self.c, self.L1, self.L2
        b = float(np.linalg.norm(S - T))
        a = _unit(S - T)                             # 末端 → 肩
        if a is None:
            return []
        e_h = _unit(np.cross(up, a))                 # 水平、⊥ S→T
        if e_h is None:                              # S→T 正好竖直:随便取一个水平方向
            e_h = _unit(np.cross(outward, a)) if _unit(np.cross(outward, a)) is not None else np.array([1.0, 0, 0])
        if np.dot(e_h, outward) < 0:
            e_h = -e_h                               # 往外那一侧
        e_v = np.cross(a, e_h)
        if np.dot(e_v, up) < 0:
            e_v = -e_v                               # 往上那一侧

        d_90, d_straight = math.hypot(L1, L2), L1 + L2
        lo, hi = abs(b - c), b + c                   # 三角形 S-W-T 能成立的 d 范围
        out = []
        for frac in self.D_FRACTION:
            d = d_90 + frac * (0.97 * d_straight - d_90)
            d = min(max(d, lo + 1e-4), hi - 1e-4)
            if not lo < d < hi or d > 0.99 * d_straight:
                continue
            cos_t = (b * b + c * c - d * d) / (2 * b * c)
            theta = math.acos(min(1.0, max(-1.0, cos_t)))
            for phi_deg in self.PHI_DEG:
                phi = math.radians(phi_deg)
                e = math.cos(phi) * e_h + math.sin(phi) * e_v
                W = T + c * (math.cos(theta) * a + math.sin(theta) * e)
                z = _unit(T - W)                     # 工具轴:腕心 → 末端
                n = _unit(np.cross(a, e))            # 基面法向
                if z is None or n is None:
                    continue
                if R_now is not None:
                    R = roll_locked(R_now, z, roll_deg)
                else:
                    x = _unit(np.cross(n, z))
                    R = np.column_stack([x, np.cross(z, x), z])
                E = self._elbow_target(W, n, outward, up)
                if E is not None:
                    out.append({"phi_deg": phi_deg, "d": round(d, 4), "W": W, "R": R, "E": E})
        return out

    def _elbow_target(self, W, n, outward, up):
        """肘在 S-W 所在、法向为 n 的面里;三角形 S-E-W 两边 L1、L2;取往外 / 往下的一侧。"""
        SW = W - self.S
        d = float(np.linalg.norm(SW))
        if not abs(self.L1 - self.L2) < d < self.L1 + self.L2:
            return None
        w = SW / d
        alpha = math.acos((self.L1 ** 2 + d * d - self.L2 ** 2) / (2 * self.L1 * d))
        m = _unit(np.cross(n, w))                    # 面内、⊥ S→W
        if m is None:
            return None
        if np.dot(m, outward) - np.dot(m, up) < 0:
            m = -m
        return self.S + self.L1 * (math.cos(alpha) * w + math.sin(alpha) * m)

    # ---- IK + 肘 + 打分 ----
    def score(self, q, R=None, tool_hint=None, hint_weight=0.0):
        """越小越舒服:各关节离限位中点的归一化距离平方和;给了 tool_hint 时,工具轴偏离它的角度也算进去。"""
        cost = float(np.sum(((np.asarray(q) - self.mid) / self.half) ** 2))
        if tool_hint is not None and R is not None and hint_weight:
            cost += hint_weight * math.acos(min(1.0, max(-1.0, float(np.dot(R[:, 2], tool_hint)))))
        return cost

    def _pull_elbow(self, q, p, R, E, iters=60, gain=0.5):
        """零空间里把肘拉向 E*(末端位姿不动),每步再修一次末端。"""
        q = np.array(q, dtype=float)
        kin, ek = self.kin, self.elbow_kin
        for _ in range(iters):
            pe = ek.fk(q[:ek.n])[0]
            err = E - pe
            if float(np.linalg.norm(err)) < 0.005:
                break
            Je = np.zeros((3, kin.n))
            Je[:, :ek.n] = ek.jacobian(q[:ek.n])[:3]
            J = kin.jacobian(q)
            N = np.eye(kin.n) - dls_pinv(J) @ J
            dq = N @ (Je.T @ err) * gain
            step = float(np.max(np.abs(dq)))
            if step > 0.05:
                dq *= 0.05 / step
            q = np.clip(q + dq, kin.lower, kin.upper)
        fine = polish(kin, q, p, R, 30)              # 零空间是线性近似,末端会漂一点,精修回去
        return fine if fine is not None else q

    def solve(self, T, q0, up, outward, valid=None, tool_hint=None, hint_weight=0.0, roll_deg=0.0):
        """返回 (最舒服的 {"q", "R", "phi_deg", "d", "score"} 或 None, 试了几个候选)。roll 锁在当前姿态上(+roll_deg)。"""
        best, tried = None, 0
        R_now = self.kin.fk(q0)[1]
        for cand in self.candidates(T, up, outward, R_now, roll_deg):
            tried += 1
            R = cand["R"]
            q = self.solver.ik(T, "full", R, R[:, 0], q0, valid=valid)
            if q is None:
                continue
            q = self._pull_elbow(q, T, R, cand["E"])
            if not self.solver.accept(q, T, "full", R, R[:, 0]):
                continue
            if valid is not None and not valid(q):
                continue
            s = self.score(q, R, tool_hint, hint_weight)
            if best is None or s < best["score"]:
                best = {"q": q, "R": R, "phi_deg": cand["phi_deg"], "d": cand["d"], "score": round(s, 3)}
        return best, tried


# ---------------------------------------------------------------- 转不到就后撤再转
def tilt_toward(R_now, goal_axis, fraction, roll_deg=0.0):
    """工具轴从当前方向往 goal_axis 转 fraction(0~1)那么多,roll 锁住(不自转,+roll_deg)。"""
    z_now = R_now[:, 2]
    goal = _unit(np.asarray(goal_axis, float))
    full = rotation_from_to(z_now, goal)
    angle = math.acos(min(1.0, max(-1.0, float(np.dot(z_now, goal)))))
    if angle < 1e-9:
        z = goal
    else:
        axis = _unit(np.cross(z_now, goal))
        if axis is None:                              # 正好反向:用 full 的转轴
            axis = _unit(np.array([full[2, 1] - full[1, 2], full[0, 2] - full[2, 0], full[1, 0] - full[0, 1]]))
            if axis is None:
                axis = _unit(np.cross(z_now, [1.0, 0, 0])) if abs(z_now[0]) < 0.9 else _unit(np.cross(z_now, [0, 1.0, 0]))
        z = _axis_angle(axis, fraction * angle) @ z_now
    return roll_locked(R_now, z, roll_deg)


def tilt_search(solver, q0, p_start, R_now, goal_axis, retreat_dir, max_retreat_m, step_m, roll_deg=0.0,
                valid=None, bisect=5):
    """“让夹爪朝下”够不着时自己想办法:原地先转到能转的最大程度,不行就沿 retreat_dir 后撤一步再转……

    后撤距离 r = 0, step, 2·step … max_retreat_m:每个 r 先试完全转到 goal_axis;不行就二分找最多能转几成。
    哪一步完全转到了就停(撤得最少);都转不全就取转得最多的那个(一样多取撤得少的)。
    只看 IK 有没有解(碰撞交给后面的规划器)。返回 {"retreat_m", "fraction", "p", "R", "q"} 或 None。"""
    retreat_dir = _unit(np.asarray(retreat_dir, float))
    best = None
    seed = np.asarray(q0, float)
    n = int(round(max_retreat_m / step_m))
    for k in range(n + 1):
        r = k * step_m
        p = np.asarray(p_start, float) + r * retreat_dir
        R = tilt_toward(R_now, goal_axis, 1.0, roll_deg)
        q = solver.ik(p, "full", R, R[:, 0], seed, valid=valid)
        if q is not None:
            return {"retreat_m": round(r, 3), "fraction": 1.0, "p": p, "R": R, "q": q}
        low, q_low, high = 0.0, None, 1.0
        for _ in range(bisect):
            mid = (low + high) / 2
            R_mid = tilt_toward(R_now, goal_axis, mid, roll_deg)
            q = solver.ik(p, "full", R_mid, R_mid[:, 0], seed, valid=valid)
            if q is not None:
                low, q_low = mid, q
            else:
                high = mid
        if q_low is not None:
            seed = q_low                                 # 下一步从这里接着找,构型连续
            if best is None or low > best["fraction"] + 1e-6:
                best = {"retreat_m": round(r, 3), "fraction": round(low, 3), "p": p,
                        "R": tilt_toward(R_now, goal_axis, low, roll_deg), "q": q_low}
    return best


# ---------------------------------------------------------------- 手臂够不着:动腰帮忙
def body_assist_search(body_kin, solver, q_body, q_arm, target_base, tool_axis=None, tolerance_deg=15.0,
                       roll_deg=0.0, lean_deg=(5, 10, 15, 20, 25, 30), turn_deg=(0, 10, -10, 20, -20),
                       body_limits=None):
    """手臂在现在的腰姿态下够不着 target_base(base_link):找改动最小的腰姿态,让它够得着。

    只改 body 第 3 关节(前倾,正 = 往前)和第 5 关节(转腰,正 = 向左);按 |前倾| + |转腰| 从小到大试。
    “够得着”= 不考虑碰撞时 IK 有解:给了 tool_axis 就要求工具轴离它 ≤ tolerance_deg(roll 锁住),
    没给就只看位置。碰撞交给之后真正的规划。
    q_body:腰 5 个关节当前角(弧度,顺序同 body_kin.joint_names);q_arm:这条手臂当前角。
    返回 {"q_body", "lean_deg", "turn_deg"} 或 None。"""
    q_body = np.asarray(q_body, float)
    q_arm = np.asarray(q_arm, float)
    target_base = np.asarray(target_base, float)
    pb0, Rb0 = body_kin.fk(q_body)
    R_now_base = Rb0 @ solver.kin.fk(q_arm)[1]           # 手腕现在在 base_link 里的朝向(腰动了它会跟着变)
    combos = sorted(((lean, turn) for lean in lean_deg for turn in turn_deg), key=_assist_cost)
    eye = np.eye(3)
    for lean, turn in combos:
        q = q_body.copy()
        q[2] += math.radians(lean)
        q[4] += math.radians(turn)
        if body_limits is not None and not all(lo <= v <= hi for v, (lo, hi) in zip(q, body_limits)):
            continue
        pb, Rb = body_kin.fk(q)
        T = Rb.T @ (target_base - pb)
        if tool_axis is None:
            ok = solver.ik(T, "pos", eye, eye[:, 0], q_arm) is not None
        else:
            R_now_root = Rb.T @ R_now_base
            found = tilt_search(solver, q_arm, T, R_now_root, Rb.T @ np.asarray(tool_axis, float),
                                Rb.T @ np.array([1.0, 0, 0]), 0.0, 1.0, roll_deg)
            ok = found is not None and _miss_deg(Rb @ found["R"], tool_axis) <= tolerance_deg
        if ok:
            return {"q_body": q, "lean_deg": lean, "turn_deg": turn}
    return None


def _assist_cost(combo):
    return abs(combo[0]) + abs(combo[1])


def _miss_deg(R_base, tool_axis):
    goal = _unit(np.asarray(tool_axis, float))
    return math.degrees(math.acos(min(1.0, max(-1.0, float(np.dot(R_base[:, 2], goal))))))
