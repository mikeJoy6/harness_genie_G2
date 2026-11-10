"""所有末端空间规划器共用的"一段"验证、目标检查、姿态调度、捷径平滑（原 ee_search.py 拆出来的）。

一段(edge) = 从某个关节角出发，积分笛卡尔把末端走直线到下一个点。验证顺序（便宜的先做）：
  a. 障碍粗筛：末端沿线段每半格采样，离障碍点 < clearance 就拒；起点/目标附近 endpoint_free 内不筛（要能贴近物体抓）。
  b. 积分笛卡尔(ArmSolver.solve)走这一段，要到位。迭代上限按这一段的长度给（够走 3 倍还多 30 步），
     走不到的段几毫秒就放弃 —— 以前固定 800 步，一条走不到的 5cm 段要白算 ~28ms，A* 的 8s 预算大半耗在这上面。
  b'. 积分笛卡尔走不到（关节限位/奇异附近，直线本身走不通）→ 局部关节插值兜底：以 q_from 为种子解终点 IK，
     关节线性插值过去；末端离这段直线 ≤ max(3cm, 15%段长)、关节总转角过 path_sane 才收。不会抡大圈。
  b''. 中途路点(不是目标)还走不到 → 放开姿态再试：先只保末端 X 轴(工具轴)朝向、滚转随意，再不行只要位置。
     最后一段(到目标)必须满足目标姿态。原因：剩下的失败几乎全是"IK 拒"——中途姿态被锁成起终点 slerp，
     那一段在这个姿态下走不通；真机没装手，中途姿态无所谓。
  e. 姿态引导(posture_pull)：7 自由度有冗余，末端沿路线走的同时，在零空间里把手臂构型往"目标处离 q0 最近的不撞 IK 解"
     拉（每步 posture_gain × 差值，不影响末端）。治的是"末端路线是通的、肘撞了"：只控末端时肘停在 DLS 给的地方，
     障碍要求换个肘的姿态才过得去。1 = 只在这一段撞了时带引导重走一次；2 = 每一段都带。
  c. path_sane 防大回环。
  d. 整臂碰撞：每个积分点都查，相邻两点关节变化 > check_step 再插值查（含环境 octomap，由调用方提供）。
每段都从上一段的【实际】关节角接着走，构型全程连续 —— 这是不抡大圈的根本保证。
"""
import itertools
import math
import time

import numpy as np

from ..algorithms import (path_sane, rotvec, interp, limit_step, LM_ANG_W, LIN_STEP, ROT_STEP, STEP_JOINT_MAX,
                          POS_TOL, ROT_TOL)
from ..kin import dls_pinv

DIRS26 = np.array([d for d in itertools.product((-1, 0, 1), repeat=3) if d != (0, 0, 0)], dtype=float)


def _exp_so3(w):
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def slerp(R0, R1, s):
    """R0 → R1 的球面插值，s∈[0,1]。"""
    return R0 @ _exp_so3(s * rotvec(R0.T @ R1))


# 一段验证的默认参数；planner_bench --edge key=value 可整体覆盖做对比
EDGE_DEFAULTS = dict(res=0.05, clearance=0.10, endpoint_free=0.12, check_step=math.radians(2), max_edges=1500,
                     joint_fallback=True, dev_abs=0.03, dev_rel=0.15, relax_orient=True, joint_direct=True,
                     posture_pull=2, posture_gain=0.1)


class EdgeChecker:
    def __init__(self, req, **kw):
        o = {**EDGE_DEFAULTS, **kw}
        res, clearance, endpoint_free, check_step = o["res"], o["clearance"], o["endpoint_free"], o["check_step"]
        max_edges, joint_fallback, dev_abs, dev_rel = o["max_edges"], o["joint_fallback"], o["dev_abs"], o["dev_rel"]
        self.relax_orient, self.joint_direct = o["relax_orient"], o["joint_direct"]
        self.posture_pull, self.posture_gain = int(o["posture_pull"]), o["posture_gain"]
        self.req, self.S, self.kin = req, req.solver, req.solver.kin
        self.tree = None
        if req.obstacles is not None and len(req.obstacles):
            from scipy.spatial import cKDTree
            self.tree = cKDTree(np.asarray(req.obstacles, dtype=float))
        self.res, self.clearance, self.endpoint_free = res, clearance, endpoint_free
        self.check_step, self.max_edges = check_step, max_edges
        self.joint_fallback, self.dev_abs, self.dev_rel = joint_fallback, dev_abs, dev_rel
        self.stats = dict(edges=0, costly=0, prefilter_reject=0, ik_reject=0, sane_reject=0,
                          collision_reject=0, valid_calls=0, joint_edges=0, relaxed_edges=0,
                          posture_edges=0)
        self.last_reject = None                       # 最近一次 edge 失败的原因：prefilter/ik/sane/collision
        self.q_goal = None                            # goal_check 找到的离 q0 最近的不撞 IK 解
        self._vcache = {}
        self.t0 = time.monotonic()
        self.q0 = np.asarray(req.q0, dtype=float)
        self.p_start, self.R_start = self.kin.fk(self.q0)
        self.p_goal, self.R_goal = np.asarray(req.p_goal, dtype=float), req.R_goal
        self.total = float(np.linalg.norm(self.p_goal - self.p_start))

    # ── 预算 ──────────────────────────────────────────────────────────
    def elapsed(self):
        return time.monotonic() - self.t0

    def out_of_budget(self):
        return self.stats["costly"] >= self.max_edges or self.elapsed() > self.req.time_limit

    def ms(self):
        return f"{self.elapsed()*1000:.0f}ms"

    # ── 碰撞 ──────────────────────────────────────────────────────────
    def valid(self, q):
        key = tuple(np.round(q, 3))
        if key not in self._vcache:
            self.stats["valid_calls"] += 1
            self._vcache[key] = bool(self.req.valid(q))
        return self._vcache[key]

    def clearance_at(self, p):
        """末端点离最近障碍点的距离（没有障碍点时 inf）。"""
        return float(self.tree.query(p)[0]) if self.tree is not None else math.inf

    def clear_line(self, pa, pb):
        if self.tree is None:
            return True
        n = max(1, int(math.ceil(np.linalg.norm(pb - pa) / (self.res / 2))))
        for s in np.linspace(0.0, 1.0, n + 1):
            p = pa + s * (pb - pa)
            if (np.linalg.norm(p - self.p_start) < self.endpoint_free or
                    np.linalg.norm(p - self.p_goal) < self.endpoint_free):
                continue
            if self.tree.query(p)[0] < self.clearance:
                return False
        return True

    def joint_path_clear(self, path):
        """关节路径逐点 + 插值查碰撞（不查第一个点：它就是上一段的终点）。"""
        for qa, qb in zip(path[:-1], path[1:]):
            n = max(1, int(math.ceil(float(np.max(np.abs(qb - qa))) / self.check_step)))
            for s in np.linspace(0.0, 1.0, n + 1)[1:]:
                if not self.valid(qa + s * (qb - qa)):
                    return False
        return True

    # ── 一段 ──────────────────────────────────────────────────────────
    def edge(self, q_from, pa, pb, Rb):
        """从关节角 q_from(末端在 pa) 积分笛卡尔到 (pb, Rb)。成功返回关节路径，失败返回 None。"""
        self.stats["edges"] += 1
        if not self.clear_line(pa, pb):
            self.stats["prefilter_reject"] += 1
            self.last_reject = "prefilter"
            return None
        self.stats["costly"] += 1                     # 粗筛之后才算"昂贵"的验证（IK + 碰撞），上限只数这个
        Ra = self.kin.fk(q_from)[1]
        lin, ang = float(np.linalg.norm(pb - pa)), float(np.linalg.norm(rotvec(Rb @ Ra.T)))
        iters = int(3 * (lin / LIN_STEP + ang / ROT_STEP)) + 30
        pull = self.posture_pull == 2 and self.q_goal is not None
        path, ok = self._solve(q_from, pb, Rb, iters, pull)
        full_ok = ok and not pull                     # 普通积分笛卡尔走通了（撞了才值得带引导重走）
        if pull and ok:
            self.stats["posture_edges"] += 1
        if not ok:
            path = self._joint_edge(q_from, pa, pb, Rb) if self.joint_fallback else None
            if path is not None:
                self.stats["joint_edges"] += 1
            elif self.relax_orient and float(np.linalg.norm(pb - self.p_goal)) > 1e-6:
                for mode in ("axis", "pos"):               # 中途路点：先只保工具轴，再只要位置
                    path, ok, _, _, _ = self.S.solve(q_from, pb, mode, Rb, Rb[:, 0], max_iters=int(3 * lin / LIN_STEP) + 30)
                    if ok:
                        self.stats["relaxed_edges"] += 1
                        ang = 0.0                             # 姿态没按 Rb 走，path_sane 只按平移算
                        break
                else:
                    path = None
            if path is None:
                self.stats["ik_reject"] += 1
                self.last_reject = "ik"
                return None
        dist = lin + LM_ANG_W * ang
        if path_sane(path, dist, ""):
            self.stats["sane_reject"] += 1
            self.last_reject = "sane"
            return None
        if not self.joint_path_clear(path):
            if self.posture_pull == 1 and self.q_goal is not None and full_ok:
                path2, ok2 = self._solve(q_from, pb, Rb, iters, True)
                if ok2 and not path_sane(path2, dist, "") and self.joint_path_clear(path2):
                    self.stats["posture_edges"] += 1
                    return path2
            self.stats["collision_reject"] += 1
            self.last_reject = "collision"
            return None
        return path

    def _solve(self, q_from, pb, Rb, iters, pull):
        """积分笛卡尔（同 ArmSolver.solve 的 full 模式）；pull=True 时零空间里往 self.q_goal 拉构型。"""
        if not pull:
            path, ok, _, _, _ = self.S.solve(q_from, pb, "full", Rb, None, max_iters=iters)
            return path, ok
        kin = self.kin
        q = np.array(q_from, dtype=float)
        path = [q.copy()]
        for _ in range(iters):
            pos, R = kin.fk(q)
            dp, drot = pb - pos, rotvec(Rb @ R.T)
            ep, er = float(np.linalg.norm(dp)), float(np.linalg.norm(drot))
            if ep < POS_TOL and er < ROT_TOL:
                return path, True
            dp = dp * min(1.0, LIN_STEP / max(ep, 1e-12))
            drot = drot * min(1.0, ROT_STEP / max(er, 1e-12))
            J = kin.jacobian(q)
            qn = limit_step(q, J, np.concatenate([dp, drot]), kin.lower, kin.upper)
            dn = (np.eye(kin.n) - dls_pinv(J) @ J) @ (self.posture_gain * (self.q_goal - q))
            dn *= min(1.0, 0.03 / max(float(np.max(np.abs(dn))), 1e-12))     # 引导每步最多 0.03rad
            q = np.clip(qn + dn, kin.lower, kin.upper)
            path.append(q.copy())
        return path, False

    def _joint_edge(self, q_from, pa, pb, Rb):
        """局部关节插值兜底：种子 = q_from 的终点 IK，关节线性插值；末端偏离这段直线太多就不要。"""
        q_b = self.S.lm(q_from, pb, "full", Rb, None)
        if q_b is None or not self.S.accept(q_b, pb, "full", Rb, None):
            return None
        path = interp(q_from, q_b, STEP_JOINT_MAX / 2)
        u = pb - pa
        L = float(np.linalg.norm(u))
        u = u / L if L > 1e-9 else u
        tol = max(self.dev_abs, self.dev_rel * L)
        for q in path[1:-1]:
            d = self.kin.fk(q)[0] - pa
            if float(np.linalg.norm(d - np.dot(d, u) * u)) > tol:
                return None
        return path

    # ── 目标 / 姿态 ───────────────────────────────────────────────────
    def goal_check(self):
        """目标多种子 IK 可达、且有不撞的 IK 解。返回 None(可以搜) 或失败原因。"""
        if self.S.ik(self.p_goal, "full", self.R_goal, None, self.q0, n=60) is None:
            return f"目标不可达：多种子 IK 解不出（{self.ms()}），不搜索"
        self.q_goal = self.S.ik(self.p_goal, "full", self.R_goal, None, self.q0, n=60, valid=self.valid)
        if self.q_goal is None:
            return f"目标姿态本身会撞：所有 IK 解都和障碍/自身碰撞（{self.ms()}），不搜索"
        return None

    def R_at(self, p):
        """姿态调度：按离目标的剩余距离在起点姿态与目标姿态之间插值，到目标正好是目标姿态。"""
        s = 1.0 - float(np.linalg.norm(p - self.p_goal)) / max(self.total, 1e-9)
        return slerp(self.R_start, self.R_goal, min(1.0, max(0.0, s)))

    def direct(self):
        """直线到目标。笛卡尔直线因为运动学走不通（不是撞）时，关节直连到离 q0 最近的不撞 IK 解
        （同 plan_pose 的关节直连；逐 2° 查碰撞、过 path_sane 防大回环）。"""
        path = self.edge(self.q0, self.p_start, self.p_goal, self.R_goal)
        if path is not None or not self.joint_direct or self.last_reject != "ik" or self.q_goal is None:
            return path
        path = interp(self.q0, self.q_goal, STEP_JOINT_MAX / 2)
        if path_sane(path, self.total, "") or not self.joint_path_clear(path):
            return None
        self.stats["joint_direct"] = 1
        return path

    # ── 平滑 ──────────────────────────────────────────────────────────
    def shortcut(self, chain):
        """chain = [(q, p), ...] 已验证的路点链（首=起点，尾=目标）。从当前点贪心直连能连上的最远路点。
        返回 (关节路径, 末端路点) 或 (None, None)。"""
        joint_path, wps = [chain[0][0]], [chain[0][1]]
        i = 0
        while i < len(chain) - 1:
            for j in range(len(chain) - 1, i, -1):
                pj = chain[j][1]
                Rj = self.R_goal if j == len(chain) - 1 else self.R_at(pj)
                seg = self.edge(joint_path[-1], wps[-1], pj, Rj)
                if seg is not None:
                    joint_path += list(seg[1:])
                    wps.append(pj)
                    i = j
                    break
            else:                                     # 相邻路点在搜索时已验证过，理论上不会
                return None, None
        return joint_path, wps

    def length(self, wps):
        return sum(float(np.linalg.norm(b - a)) for a, b in zip(wps[:-1], wps[1:]))

    def stat_text(self):
        s = self.stats
        return (f"验证 {s['edges']} 段(其中 IK+碰撞 {s['costly']} 段)：粗筛拒 {s['prefilter_reject']}、IK 拒 {s['ik_reject']}、"
                f"回环拒 {s['sane_reject']}、碰撞拒 {s['collision_reject']}，关节兜底 {s['joint_edges']} 段、"
                f"放开姿态 {s['relaxed_edges']} 段、姿态引导 {s['posture_edges']} 段，"
                f"碰撞查询 {s['valid_calls']} 次")

    def done_text(self, kind, wps, raw_n=None):
        raw = f"{raw_n} 点 → " if raw_n else ""
        return (f"{kind}：{raw}{len(wps)-1} 段直线，末端路程 {self.length(wps)*100:.0f}cm"
                f"（直线距离 {self.total*100:.0f}cm；{self.stat_text()}，{self.ms()}）")
