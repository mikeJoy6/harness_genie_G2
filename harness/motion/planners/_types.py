"""绕障规划器的约定：纯计算，不碰 ROS、不发指令；碰撞检查以回调传入（真机 = MoveIt /check_state_validity）。"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import numpy as np


@dataclass
class PlanRequest:
    """全部在手臂链根系(arm_base_link)。"""
    solver: Any                                   # algorithms.ArmSolver
    valid: Callable[[np.ndarray], bool]           # 整机碰撞检查(含 octomap)：True = 不撞
    q0: np.ndarray                                # 当前关节角
    p_goal: np.ndarray
    R_goal: np.ndarray
    obstacles: Optional[np.ndarray] = None        # (N,3) octomap 占据体素中心，粗筛/离障距离用，可为 None
    up: np.ndarray = field(default_factory=lambda: np.array([0.0, -1.0, 0.0]))   # 根系里的"上"(base_link +z)
    time_limit: float = 8.0
    log: Callable[[str], None] = lambda s: None


@dataclass
class PlanResult:
    joint_path: Optional[list]                    # 关节路径(每步 ≤ STEP_JOINT_MAX)，失败为 None
    waypoints: Optional[list]                     # 末端路点（展示/统计用）
    note: str
    stats: dict = field(default_factory=dict)

    @property
    def ok(self):
        return self.joint_path is not None


class Planner(ABC):
    @abstractmethod
    def plan(self, req: PlanRequest) -> PlanResult:
        """规划 q0 → (p_goal, R_goal)。只返回路径，不执行。"""
