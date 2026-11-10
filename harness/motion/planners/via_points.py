"""候选绕行模板：不搜索，直接枚举少量几何模板，按末端路程从短到长逐个验证，第一个通过的就是模板里最短的。

模板（d = 起点→目标，n = 垂直于 d 平面内的 8 个方向，"上"排第一，h = 绕开距离）：
  V  一个中途点：  起点 → 中点+h·n → 目标
  梯形 两个中途点：起点 → 1/4处+h·n → 3/4处+h·n → 目标
  U  两个中途点：  起点 → 起点+h·n → 目标+h·n → 目标   （先抬起、平移、再落下）
每个模板先只做末端粗筛（毫秒级），通过了才走积分笛卡尔+碰撞。模板都不行 = 失败（不兜底）。
"""
import numpy as np

from ._edges import EdgeChecker
from ._registry import register
from ._types import Planner, PlanResult

HEIGHTS = (0.08, 0.12, 0.18, 0.25, 0.33, 0.42)


def candidates(E, up):
    d = E.p_goal - E.p_start
    L = float(np.linalg.norm(d))
    u = d / L if L > 1e-6 else np.array([1.0, 0.0, 0.0])
    a = up - np.dot(up, u) * u                     # "上"在垂直平面里的投影
    if np.linalg.norm(a) < 1e-3:
        a = np.cross(u, [1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.cross(u, [0.0, 1.0, 0.0])
    a /= np.linalg.norm(a)
    b = np.cross(u, a)
    dirs = [np.cos(t) * a + np.sin(t) * b for t in np.radians([0, 45, -45, 90, -90, 135, -135, 180])]
    out = []
    for h in HEIGHTS:
        for n in dirs:
            off = h * n
            out.append(("V", [E.p_start + 0.5 * d + off]))
            out.append(("梯形", [E.p_start + 0.25 * d + off, E.p_start + 0.75 * d + off]))
            out.append(("U", [E.p_start + off, E.p_goal + off]))
    pts = lambda vias: [E.p_start] + vias + [E.p_goal]
    out.sort(key=lambda c: E.length(pts(c[1])))
    return [(name, pts(vias)) for name, vias in out]


def try_templates(E, req, max_templates=None):
    """返回 (关节路径, 路点, 模板名) 或 (None, None, None)。"""
    tried = 0
    for name, wps in candidates(E, np.asarray(req.up, dtype=float)):
        if E.out_of_budget() or (max_templates and tried >= max_templates):
            break
        if not all(E.clear_line(a, b) for a, b in zip(wps[:-1], wps[1:])):
            continue
        tried += 1
        jp = [E.q0]
        for i, (a, b) in enumerate(zip(wps[:-1], wps[1:])):
            R = E.R_goal if i == len(wps) - 2 else E.R_at(b)
            seg = E.edge(jp[-1], a, b, R)
            if seg is None:
                break
            jp += list(seg[1:])
        else:
            return jp, wps, name
    return None, None, None


@register("via_points", "绕行模板(V/梯形/U × 8 方向 × 6 高度)按路程排序逐个验证，不搜索")
class ViaPoints(Planner):
    def plan(self, req):
        E = EdgeChecker(req)
        why = E.goal_check()
        if why:
            return PlanResult(None, None, why, E.stats)
        direct = E.direct()
        if direct is not None:
            return PlanResult(direct, [E.p_start, E.p_goal], f"直线可达（{E.ms()}）", E.stats)
        jp, wps, name = try_templates(E, req)
        if jp is None:
            return PlanResult(None, None, f"模板都不通（{E.stat_text()}，{E.ms()}）", E.stats)
        return PlanResult(jp, wps, E.done_text(f"{name} 型绕行", wps), E.stats)
