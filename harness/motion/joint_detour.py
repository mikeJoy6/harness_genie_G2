"""joint_detour.py —— 全身关节点到点的保底绕行（goto 直线插值会撞时才用）。

为什么不用 genie_robot_agibot_g2_motion 的末端绕障（planners/）：那是单臂、目标是末端位姿、腰固定，
  而且刻意从当前构型连续走、不换分支（防大回环的原理）。录好的位姿常常在另一个分支（2026-11-04 find：
  左臂 j1 要转 +201°、j3 −161°），它结构上到不了那组关节。这里目标就是一组关节角，换分支是必须的。

按顺序试（都是关节空间折线，每段沿途每 step 弧度问一次 MoveIt，整机 + 点云 + 附着物）：
  1. 经一个录好的位姿中转：q0 → 中转 → 目标，两段都不撞；多个可行取关节总行程最小的。
     中转点只取该位姿里"这次要动的关节"，其余保持当前。
  2. 关节空间 RRT-Connect（只在要动的关节上采样）：每个关节的采样范围 = [起止较小值 − pad, 较大值 + pad] ∩ URDF 限位，
     不让它去不相干的地方抡；找到后随机捷径平滑。时间预算内找不到就放弃。
返回折线路点（不含起点）或 None。
"""
import math
import time

import numpy as np


class JointDetour:
    def __init__(self, valid, limits, poses=None, step=0.05, budget_s=15.0, pad=0.5, log=None, cancelled=None):
        """valid({关节: 值}) → (ok, 原因)；limits {关节: (lo, hi)}；poses {名字: {关节: 值}}（中转候选）。"""
        self.valid, self.limits, self.poses = valid, limits, poses or {}
        self.step, self.budget, self.pad = step, budget_s, pad
        self.log = log or (lambda _s: None)
        self.cancelled = cancelled
        self.n_checks = 0

    def _ok(self, names, x):
        self.n_checks += 1
        return self.valid(dict(zip(names, (float(v) for v in x))))[0]

    def _seg(self, names, a, b):
        n = max(1, int(math.ceil(float(np.max(np.abs(b - a))) / self.step)))
        return all(self._ok(names, a + (b - a) * k / n) for k in range(1, n + 1))

    def plan(self, q0, target):
        names = list(target)
        a = np.array([q0[j] for j in names])
        b = np.array([target[j] for j in names])
        t0 = time.monotonic()
        path = self._via(names, a, b)
        if path is not None:
            return [dict(zip(names, map(float, p))) for p in path], path_note(self, "经录好位姿中转", t0)
        path = self._rrt(names, a, b, t0)
        if path is not None:
            return [dict(zip(names, map(float, p))) for p in path], path_note(self, "关节 RRT-Connect", t0)
        return None, f"中转和 RRT 都没找到不撞的路（{time.monotonic() - t0:.1f}s，碰撞检查 {self.n_checks} 次）"

    # ── 1. 经录好的位姿中转 ─────────────────────────────────────
    def _via(self, names, a, b):
        cands = []
        for name, joints in self.poses.items():
            v = np.array([joints.get(j, a[i]) for i, j in enumerate(names)])
            if np.max(np.abs(v - a)) < 1e-3 or np.max(np.abs(v - b)) < 1e-3:
                continue
            cands.append((float(np.abs(v - a).sum() + np.abs(b - v).sum()), name, v))
        for _, name, v in sorted(cands, key=lambda c: c[0]):
            if self._stop():
                return None
            if self._seg(names, a, v) and self._seg(names, v, b):
                self.log(f"经 '{name}' 中转")
                return [v, b]
        return None

    # ── 2. RRT-Connect ─────────────────────────────────────────
    def _rrt(self, names, a, b, t0, grow=0.3):
        lo = np.minimum(a, b) - self.pad
        hi = np.maximum(a, b) + self.pad
        for i, j in enumerate(names):
            if j in self.limits:
                lo[i], hi[i] = max(lo[i], self.limits[j][0]), min(hi[i], self.limits[j][1])
        fixed = np.abs(b - a) < 1e-3                     # 不动的关节不采样
        lo[fixed], hi[fixed] = a[fixed], a[fixed]
        rng = np.random.default_rng(0)
        trees = [([a], [-1]), ([b], [-1])]

        def extend(tree, x):
            V, par = tree
            i = int(np.argmin([np.max(np.abs(v - x)) for v in V]))
            d = x - V[i]
            n = float(np.max(np.abs(d)))
            qn = V[i] + d * min(1.0, grow / max(n, 1e-9))
            if not self._seg(names, V[i], qn):
                return None
            V.append(qn)
            par.append(i)
            return len(V) - 1

        def trace(tree, k):
            V, par = tree
            out = []
            while k != -1:
                out.append(V[k])
                k = par[k]
            return out[::-1]

        A, B = 0, 1
        while time.monotonic() - t0 < self.budget and not self._stop():
            x = rng.uniform(lo, hi)
            j = extend(trees[A], x)
            if j is not None:
                y = trees[A][0][j]
                while True:                               # connect：另一棵树一路长过去
                    k = extend(trees[B], y)
                    if k is None:
                        break
                    if np.max(np.abs(trees[B][0][k] - y)) < 1e-6:
                        p = trace(trees[A], j) + trace(trees[B], k)[::-1][1:]
                        if A == 1:
                            p = p[::-1]
                        return self._shortcut(names, p, rng, t0)[1:]
            A, B = B, A
        return None

    def _shortcut(self, names, p, rng, t0, tries=60):
        p = list(p)
        for _ in range(tries):
            if len(p) <= 2 or time.monotonic() - t0 > self.budget or self._stop():
                break
            i, k = sorted(rng.choice(len(p), 2, replace=False))
            if k - i > 1 and self._seg(names, p[i], p[k]):
                p = p[:i + 1] + p[k:]
        return p

    def _stop(self):
        return self.cancelled is not None and self.cancelled.is_set()


def path_note(planner, how, t0):
    return f"{how}（{time.monotonic() - t0:.1f}s，碰撞检查 {planner.n_checks} 次）"
