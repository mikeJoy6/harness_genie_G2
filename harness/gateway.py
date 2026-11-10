"""模型和机器人之间的关卡:坐标在这里计算,安全检查也在这里,模型绕不过去。"""
import math
import time


def _round(xyz):
    return [round(float(v), 4) for v in xyz]


class Gateway:
    def __init__(self, backend, motion_cfg, colors, confirm=None, plan_only=False):
        self.backend = backend
        self.cfg = motion_cfg
        self.colors = colors            # {颜色名: HSV 范围列表}
        self.confirm = confirm          # confirm(提示文字) → True/False;None 表示不询问
        self.plan_only = plan_only      # True:只规划,绝不发送运动
        self.target = None
        self.target_time = None
        self.moves = 0

    def execute(self, name, args):
        handlers = {"locate_object": self._locate,
                    "get_left_end_pose": self._end_pose,
                    "approach_object": self._approach}
        if name not in handlers:
            return {"ok": False, "error": f"没有工具 '{name}'"}
        try:
            return handlers[name](args)
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    def _locate(self, args):
        color = args.get("color")
        if color not in self.colors:
            return {"ok": False, "error": f"不认识颜色 '{color}',可选:{list(self.colors)}"}
        self.target = None              # 定位失败时不能沿用上一次的目标
        result = self.backend.locate_object(color, self.colors[color])
        if result.get("ok"):
            self.target = list(result["xyz_m"])
            self.target_time = time.monotonic()
        return result

    def _end_pose(self, _args):
        return {"ok": True, "frame": "base_link", "link": "arm_l_end_link",
                "xyz_m": _round(self.backend.left_end_xyz())}

    def _approach(self, args):
        cfg = self.cfg

        if self.target is None:
            return {"ok": False, "error": "还没有成功定位物体,先调用 locate_object"}
        age = time.monotonic() - self.target_time
        if age > cfg["detection_max_age_s"]:
            return {"ok": False, "error": f"物体位置已过期({age:.0f}s),重新调用 locate_object"}
        if self.moves >= cfg["max_moves_per_command"]:
            return {"ok": False, "error": "这条指令的运动次数已用完,请用户重新下指令"}

        standoff = args.get("standoff_cm", cfg["standoff_default_m"] * 100)
        if isinstance(standoff, bool) or not isinstance(standoff, (int, float)):
            return {"ok": False, "error": "standoff_cm 必须是数字"}
        standoff = float(standoff) / 100
        if not cfg["standoff_min_m"] <= standoff <= cfg["standoff_max_m"]:
            return {"ok": False, "error": f"standoff_cm 必须在 {cfg['standoff_min_m'] * 100:.0f} 到 "
                                          f"{cfg['standoff_max_m'] * 100:.0f} 之间"}

        target = [self.target[0], self.target[1], self.target[2] + standoff]
        for axis, value in zip("xyz", target):
            low, high = cfg["workspace"][axis]
            if not low <= value <= high:
                return {"ok": False, "error": f"目标 {axis}={value:.3f}m 超出工作范围 [{low}, {high}]"}

        start = self.backend.left_end_xyz()
        travel = math.dist(start, target)
        if travel > cfg["max_travel_m"]:
            return {"ok": False, "error": f"移动距离 {travel:.2f}m 超过上限 {cfg['max_travel_m']}m"}
        if start[2] < self.target[2] + cfg["min_start_above_block_m"]:
            return {"ok": False, "error": f"末端高度 {start[2]:.2f}m 不够高,直线靠近可能穿过桌面。"
                                          "请用户先把左臂移到桌面上方。"}

        plan = self.backend.plan_left_to(target)
        info = {"target_xyz_m": _round(target), "standoff_m": round(standoff, 3),
                "travel_m": round(travel, 3), "plan": plan["summary"] if plan["ok"] else None}
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}
        if self.plan_only:
            return {"ok": True, "executed": False, "note": "只规划模式,机器人没有动", **info}
        if self.confirm and not self.confirm(
                f"左臂末端将直线移动 {travel * 100:.1f}cm 到 {_round(target)}"
                f"(物体上方 {standoff * 100:.0f}cm),{plan['summary']}。输入 y 执行:"):
            return {"ok": False, "executed": False, "error": "用户拒绝了这次运动", **info}

        self.moves += 1
        done = self.backend.execute(plan)
        end = self.backend.left_end_xyz()
        return {"ok": done["ok"], "executed": done["ok"], "msg": done["msg"], "end_xyz_m": _round(end),
                "error_to_target_mm": round(math.dist(end, target) * 1000, 1),
                "distance_to_object_m": round(math.dist(end, self.target), 3), **info}
