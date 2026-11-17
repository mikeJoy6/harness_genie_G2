"""给场景测试用的“会动的”假机器人:在 FakeBackend 上加一点物理,让模型的验证有意义。

    闭合夹爪时,夹爪尖 3 厘米内有物体 → 拿起来:之后 find_objects 找它,坐标跟着夹爪尖走(下方 1 厘米)
    张开夹爪时,手里的物体落下:正下方 8 厘米内有别的物体(比如人手)就落在它上面,否则落到夹爪尖下方 3 厘米

FakeBackend(单元测试用)本身不改。
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness.backends.fake import FakeBackend  # noqa: E402


class FakeWorld(FakeBackend):
    def __init__(self, objects, **kwargs):
        """objects:{英文短语: [[x, y, z], ...]}。"""
        positions = {}
        for phrase, points in objects.items():
            positions[phrase] = [list(p) for p in points]
        super().__init__(sam_objects=positions, **kwargs)
        self.held = {"left": None, "right": None}       # (短语, 第几个)

    def _tip(self, arm):
        return self.end_pose(arm)["xyz_m"]

    def find_objects(self, query, suffix=""):
        for arm, held in self.held.items():             # 拿着的东西跟着夹爪走
            if held is not None:
                phrase, index = held
                tip = self._tip(arm)
                self.sam_objects[phrase][index] = [tip[0], tip[1], tip[2] - 0.01]
        return super().find_objects(query, suffix)

    def gripper_command(self, arm, action):
        result = super().gripper_command(arm, action)
        if not result.get("ok"):
            return result
        tip = self._tip(arm)
        if action == "close" and self.held[arm] is None:
            best = None
            for phrase, points in self.sam_objects.items():
                for index, p in enumerate(points):
                    gap = math.dist(p, tip)
                    if gap < 0.03 and (best is None or gap < best[0]):
                        best = (gap, phrase, index)
            if best is not None:
                self.held[arm] = (best[1], best[2])
        elif action == "open" and self.held[arm] is not None:
            phrase, index = self.held[arm]
            landing = [tip[0], tip[1], tip[2] - 0.03]
            for other_phrase, points in self.sam_objects.items():
                for other_index, p in enumerate(points):
                    if (other_phrase, other_index) == (phrase, index):
                        continue
                    below = tip[2] - p[2]
                    if math.hypot(p[0] - tip[0], p[1] - tip[1]) < 0.06 and 0 < below < 0.08:
                        landing = [p[0], p[1], p[2] + 0.02]
            self.sam_objects[phrase][index] = landing
            self.held[arm] = None
        return result
