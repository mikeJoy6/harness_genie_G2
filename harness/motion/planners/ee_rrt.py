"""末端空间 RRT（目标偏置 20%）：在起终点包围盒(+35cm)里随机撒点，从最近节点积分笛卡尔走 ≤10cm，
节点带关节角（构型连续）；离目标 ≤10cm 试连目标。最后捷径平滑。固定随机种子，结果可复现。"""
import numpy as np

from ._edges import EdgeChecker
from ._registry import register
from ._types import Planner, PlanResult

STEP = 0.10


@register("ee_rrt", "末端空间 RRT(目标偏置 0.2，步长 10cm)+捷径平滑")
class EERRT(Planner):
    def plan(self, req):
        E = EdgeChecker(req)
        why = E.goal_check()
        if why:
            return PlanResult(None, None, why, E.stats)
        direct = E.direct()
        if direct is not None:
            return PlanResult(direct, [E.p_start, E.p_goal], f"直线可达（{E.ms()}）", E.stats)
        return rrt(E)


def rrt(E, kind="末端RRT"):
    """在 E（已做完目标检查/直线）的剩余预算里跑 RRT。"""
    rng = np.random.default_rng(0)
    lo = np.minimum(E.p_start, E.p_goal) - 0.35
    hi = np.maximum(E.p_start, E.p_goal) + 0.35
    P, Q, parent = [E.p_start], [E.q0], [-1]
    goal_idx = None
    while not E.out_of_budget():
        x = E.p_goal if rng.random() < 0.2 else rng.uniform(lo, hi)
        i = int(np.argmin(np.linalg.norm(np.asarray(P) - x, axis=1)))
        d = x - P[i]
        n = float(np.linalg.norm(d))
        if n < 1e-3:
            continue
        pb = P[i] + d * min(1.0, STEP / n)
        seg = E.edge(Q[i], P[i], pb, E.R_at(pb))
        if seg is None:
            continue
        P.append(pb); Q.append(seg[-1]); parent.append(i)
        if np.linalg.norm(pb - E.p_goal) <= STEP:
            last = E.edge(seg[-1], pb, E.p_goal, E.R_goal)
            if last is not None:
                P.append(E.p_goal); Q.append(last[-1]); parent.append(len(P) - 2)
                goal_idx = len(P) - 1
                break
    if goal_idx is None:
        return PlanResult(None, None, f"{kind}失败（{len(P)} 节点；{E.stat_text()}，{E.ms()}）", E.stats)
    chain, k = [], goal_idx
    while k >= 0:
        chain.append((Q[k], P[k]))
        k = parent[k]
    chain.reverse()
    jp, wps = E.shortcut(chain)
    if jp is None:
        return PlanResult(None, None, "平滑时相邻路点连不上", E.stats)
    return PlanResult(jp, wps, E.done_text(kind, wps, len(chain)), E.stats)
