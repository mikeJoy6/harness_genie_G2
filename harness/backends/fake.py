"""假机器人:不需要 ROS,用来测试 harness 的逻辑。"""


class FakeBackend:
    def __init__(self, block=(0.55, 0.25, 0.60), end=(0.59, 0.42, 0.77), visible=("red", "green"), plan_error=None):
        self.block = list(block)
        self.end = list(end)
        self.visible = visible
        self.plan_error = plan_error
        self.executed = []

    def locate_object(self, color, ranges):
        if color not in self.visible:
            return {"ok": False, "error": f"画面里没有看到 {color} 物体"}
        return {"ok": True, "frame": "base_link", "tf": "target_object", "color": color, "xyz_m": list(self.block)}

    def left_end_xyz(self):
        return list(self.end)

    def plan_left_to(self, target):
        if self.plan_error:
            return {"ok": False, "error": self.plan_error}
        return {"ok": True, "target": list(target), "summary": "假规划",
                "lowest_z_m": min(self.end[2], target[2])}             # 直线路径的最低点在两端之一

    def execute(self, plan):
        self.executed.append(plan["target"])
        self.end = list(plan["target"])
        return {"ok": True, "msg": "假执行完成"}

    def close(self):
        pass
