"""假机器人:不需要 ROS,用来测试 harness 的逻辑。方法和 ros.py 的 RosBackend 一一对应。"""
import math

from ..motion.joints import GROUPS
from ..perception.sam3_objects import slug

ARMS = ("left", "right")


def _distance(obj):
    return obj["distance_m"]


FAKE_CAMERA = (0.20, 0.0, 1.68)    # 头部相机大概的位置(base_link,米),零位时往下看约 25°
FAKE_CAMERA_TILT_DEG = 25.0


class FakeBackend:
    def __init__(self, end=(0.59, 0.42, 0.77), plan_error=None, joints=None, limit_rad=3.0, sam_objects=None,
                 sam_error=None, grippers=None):
        # SAM 3 假结果:{英文短语: [物体坐标, ...]};sam_error 模拟服务没开
        self.sam_objects = sam_objects if sam_objects is not None else {
            "phone": [(0.75, -0.10, 0.80), (0.60, 0.30, 0.78)]}
        self.sam_error = sam_error
        self.segment_calls = []         # 每次 find_objects 的 (短语, 后缀)
        # 夹爪:{手: 型号},型号为空 = 没装(和真机 GDK 的 end_model 一样);默认两只手都装了 omnipicker
        self.grippers = grippers if grippers is not None else {"left": "omnipicker", "right": "omnipicker"}
        self.gripper_open = {"left": False, "right": False}
        self.gripper_calls = []         # 每次开合的 (手, 动作)
        self.ends = {"left": {"xyz_m": list(end), "rpy_deg": [0.0, 0.0, 0.0]},
                     "right": {"xyz_m": [end[0], -end[1], end[2]], "rpy_deg": [0.0, 0.0, 0.0]}}
        self.plan_error = plan_error
        self.joints = {j: 0.0 for _, names in GROUPS.values() for j in names}
        self.joints.update(joints or {})
        self.limits = {j: (-limit_rad, limit_rad) for j in self.joints}
        self.executed = []              # 每次执行的目标:手臂是 [x, y, z],关节是 {关节名: 弧度}

    # ---- 读 ----
    def find_objects(self, query, suffix=""):
        """和 RosBackend.find_objects 返回格式一样。偏离画面中心的像素按当前头部角度粗算。"""
        self.segment_calls.append((query, suffix))
        if self.sam_error:
            return {"ok": False, "error": self.sam_error}
        objects = []
        for xyz in self.sam_objects.get(query, []):
            yaw, pitch = self._head_angles_for(xyz)
            off_h = math.degrees(self.joints["idx11_head_joint1"] - yaw)       # 正 = 目标在右边
            off_v = math.degrees(pitch - self.joints["idx13_head_joint3"])     # 正 = 目标在下边
            objects.append({"label": query, "score": 0.9, "xyz_m": list(xyz),
                            "distance_m": round(math.dist(xyz, FAKE_CAMERA), 3),
                            "offset_from_center_px": [round(off_h * 5.3, 1), round(off_v * 5.3, 1)]})
        objects.sort(key=_distance)
        for k, obj in enumerate(objects, start=1):
            obj["id"] = f"{slug(query)}_{k}"
        return {"ok": True, "query": query, "objects": objects, "log_dir": f"traces/sam3/fake_{query}{suffix}"}

    def _head_angles_for(self, xyz):
        dx, dy, dz = (xyz[0] - FAKE_CAMERA[0], xyz[1] - FAKE_CAMERA[1], xyz[2] - FAKE_CAMERA[2])
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, math.hypot(dx, dy)) - math.radians(FAKE_CAMERA_TILT_DEG)
        return yaw, pitch

    def head_angles_to(self, xyz):
        """和 RosBackend.head_angles_to 一样的返回格式。"""
        yaw, pitch = self._head_angles_for(xyz)
        target = {"idx11_head_joint1": yaw, "idx12_head_joint2": self.joints["idx12_head_joint2"],
                  "idx13_head_joint3": pitch}
        return {"ok": True, "target": target, "residual_deg": [0.0, 0.0], "at_limit": False}

    def take_photo(self, camera, folder):
        self.photos = getattr(self, "photos", []) + [camera]
        return {"ok": True, "path": f"{folder}/fake_{camera}.jpg", "camera": camera, "width": 1920, "height": 1536}

    def shoulder_xyz(self, arm):
        return [0.165, 0.244 if arm == "left" else -0.244, 1.122]     # 真机 2026-11-13 腰姿态下的肩心

    def gripper_state(self, arm):
        return {"ok": True, "arm": arm, "model": self.grippers.get(arm, ""), "error_code": 0,
                "position": None, "opened": self.gripper_open[arm]}

    def gripper_command(self, arm, action):
        self.gripper_calls.append((arm, action))
        if not self.grippers.get(arm):
            return {"ok": False, "arm": arm, "error": f"{arm} 没有识别到夹爪(GDK {arm}_end_model 为空),可能没装或没上电"}
        self.gripper_open[arm] = action == "open"
        return {"ok": True, "arm": arm, "action": action, "verified": False, "msg": "假夹爪:命令已发送"}

    def end_pose(self, arm):
        return {"xyz_m": list(self.ends[arm]["xyz_m"]), "rpy_deg": list(self.ends[arm]["rpy_deg"]),
                "link": f"tcp_{arm[0]}_link"}

    def joint_state(self):
        return dict(self.joints)

    def joint_limits(self):
        return dict(self.limits)

    # ---- 规划(不动) ----
    def plan_arm_to(self, arm, target, rpy_deg=None, orientation=("keep",), wrist_roll_deg=0.0, tool_axis=None,
                    retreat=None, allow_body=True):
        self.last_retreat = retreat
        self.last_orientation = ("exact",) if rpy_deg is not None else tuple(orientation)
        if rpy_deg is None and tool_axis is not None:
            self.last_orientation = ("axis",)
        self.last_roll = wrist_roll_deg
        self.last_tool_axis = None if tool_axis is None else list(tool_axis)
        if self.plan_error:
            return {"ok": False, "error": self.plan_error}
        start = self.ends[arm]["xyz_m"]
        return {"ok": True, "kind": "arm", "arm": arm, "target": list(target),
                "rpy_deg": list(rpy_deg) if rpy_deg is not None else list(self.ends[arm]["rpy_deg"]),
                "summary": "假规划", "orientation": self.last_orientation[0],
                "lowest_z_m": min(start[2], target[2])}   # 直线路径的最低点在两端之一

    def plan_touch(self, arm, object_xyz, gap_m, direction, allow_body=True):
        """和 RosBackend.plan_touch 一样的返回格式:先到接触点前 5cm,再慢速直线到接触点。"""
        self.last_touch = {"arm": arm, "object_xyz": list(object_xyz), "gap_m": gap_m, "direction": list(direction)}
        if self.plan_error:
            return {"ok": False, "error": self.plan_error}
        contact = [o - gap_m * d for o, d in zip(object_xyz, direction)]
        pre = [c - 0.05 * d for c, d in zip(contact, direction)]
        return {"ok": True, "kind": "touch", "arm": arm, "contact_xyz_m": contact, "precontact_xyz_m": pre,
                "unchecked_m": 0.05, "approach": "末端→物体", "rpy_deg": list(self.ends[arm]["rpy_deg"]),
                "summary": "假规划:先到接触点前 5cm,再慢速直线 5cm"}

    def plan_joints(self, target, speed):
        if self.plan_error:
            return {"ok": False, "error": self.plan_error}
        dmax = max(abs(v - self.joints[j]) for j, v in target.items())
        return {"ok": True, "kind": "joints", "target": dict(target),
                "summary": f"假关节规划,最大关节变化 {math.degrees(dmax):.1f}°"}

    # ---- 执行 ----
    def execute(self, plan):
        if plan["kind"] in ("arm", "body_then"):        # body_then:假机器人不模拟腰,当成手臂直接到位
            self.executed.append(plan["target"])
            self.ends[plan["arm"]] = {"xyz_m": list(plan["target"]), "rpy_deg": list(plan["rpy_deg"])}
        elif plan["kind"] == "touch":
            self.executed.append(plan["contact_xyz_m"])
            self.ends[plan["arm"]] = {"xyz_m": list(plan["contact_xyz_m"]), "rpy_deg": list(plan["rpy_deg"])}
        else:
            self.executed.append(dict(plan["target"]))
            self.joints.update(plan["target"])
        return {"ok": True, "msg": "假执行完成"}

    def close(self):
        pass
