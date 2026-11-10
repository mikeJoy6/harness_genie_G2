"""末端绕障规划器，一个模块一个、互相平等（同 genie_motion/algorithms 的结构）：
加一个模块并用 @register 注册即可，不用改任何中心列表。下划线开头的是共用件，不是算法。
各算法离线对比：ros2 run genie_robot_agibot_g2_motion planner_bench（结果见 DEVLOG）。"""
from ._registry import discover, register
from ._types import Planner, PlanRequest, PlanResult

DEFAULT = "via_then_rrt"   # planner_bench 2026-11-02：3 组留出题(196 道绕障)合计最好 183/196，规划中位 0.58s；见 DEVLOG

__all__ = ["DEFAULT", "Planner", "PlanRequest", "PlanResult", "discover", "register"]
