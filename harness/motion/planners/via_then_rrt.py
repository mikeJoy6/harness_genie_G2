"""混合：先试全部绕行模板（几何上最短的先试，几十~几百毫秒），都不通再在剩余预算里跑末端空间 RRT 兜底。
模板管"常见的 U 型/翻越"，RRT 管"模板想不到的形状"（L 型通道、两侧都有东西）。两者都只走积分笛卡尔段，不抡大圈。"""
from ._edges import EdgeChecker
from ._registry import register
from ._types import Planner, PlanResult
from .ee_rrt import rrt
from .via_points import try_templates


@register("via_then_rrt", "先全部绕行模板，不通再末端 RRT 兜底(+捷径平滑)")
class ViaThenRRT(Planner):
    def plan(self, req):
        E = EdgeChecker(req)
        why = E.goal_check()
        if why:
            return PlanResult(None, None, why, E.stats)
        direct = E.direct()
        if direct is not None:
            return PlanResult(direct, [E.p_start, E.p_goal], f"直线可达（{E.ms()}）", E.stats)
        jp, wps, name = try_templates(E, req)
        if jp is not None:
            return PlanResult(jp, wps, E.done_text(f"{name} 型绕行", wps), E.stats)
        return rrt(E, kind="模板不通→末端RRT")
