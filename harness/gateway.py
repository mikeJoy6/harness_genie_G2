"""模型和机器人之间的关卡:坐标在这里计算,安全检查也在这里,模型绕不过去。

碰撞由规划器负责(整机自碰撞 + octomap 点云 + 碰撞体,会撞就绕),这里只守底线:
  · 物体只能用 find_objects(SAM 3)返回的编号指代,坐标存在 self.objects 里,模型给不了坐标
  · approach_object / point_at_object / look_at_object:目标由物体坐标算出来,全程查碰撞
  · touch_object:   唯一会碰到物体的工具,最后几厘米只放过物体附近的点云(backend.plan_touch)
  · move_arm_end:   只能给相对当前末端的偏移和转角;目标要在工作范围内
  · move_joints:    关节增量或目标角;要在 URDF 限位内,单次变化有上限(防止模型把度当成弧度之类的错)
  · go_to_pose:     只能选 configs/poses.yaml 里人录好的位姿
所有运动都是 先规划 → 检查 → (只规划模式到此为止)→ confirm(默认只打印,--confirm 时人按 y)→ 执行。
上限都在 configs/harness.yaml。
"""
import math
import time

from .geometry import rotate_rpy_deg, rotation_between_deg, rpy_deg_to_matrix
from .motion.joints import GROUPS
from .perception.sam3_objects import slug

ARMS = ("left", "right")
ARM_NAME = {"left": "左臂", "right": "右臂"}
HAND_NAME = {"left": "左手", "right": "右手"}
GROUP_NAME = {"body": "腰", "head": "头", "left_arm": "左臂", "right_arm": "右臂"}
LIMIT_MARGIN_RAD = 0.02         # 目标关节角离 URDF 限位至少留这么多


def _round(xyz):
    return [round(float(v), 4) for v in xyz]


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


TOOL_AXES = {"down": (0, 0, -1), "up": (0, 0, 1), "forward": (1, 0, 0), "backward": (-1, 0, 0),
             "left": (0, 1, 0), "right": (0, -1, 0)}     # 工具轴(末端 Z,腕心→末端)朝 base_link 的哪个方向


def _tool_axis_arg(args):
    """tool_axis:让夹爪/工具朝哪(down 竖直朝下……)。返回 (方向向量 或 None, 错误)。"""
    name = args.get("tool_axis")
    if name is None:
        return None, None
    if name not in TOOL_AXES:
        return None, f"tool_axis 只能是 {list(TOOL_AXES)}"
    return list(TOOL_AXES[name]), None


def _tool_axis_report(R_rpy_deg):
    """末端姿态 → 工具轴方向 + 离竖直朝下多少度,模型不用自己算欧拉角。"""
    R = rpy_deg_to_matrix(R_rpy_deg)
    z = R[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(z[2])))))
    return [round(float(v), 3) for v in z], round(tilt, 1)


def _plan_notes(plan):
    """规划动了腰,或者 touch 最后一段放过物体附近的点云时,把这件事写进结果,模型要如实告诉用户。"""
    notes = {}
    if plan.get("kind") == "body_then":
        notes["body_assist"] = {"lean_deg": plan["lean_deg"], "turn_deg": plan["turn_deg"]}
        notes["note"] = f"手臂够不着,先让上身前倾 {plan['lean_deg']}°、转腰 {plan['turn_deg']}° 再伸手(腰不会自己回去)"
    if plan.get("kind") == "touch":
        notes["approach"] = plan["approach"]
        notes["precontact_xyz_m"] = _round(plan["precontact_xyz_m"])
        notes["unchecked_m"] = round(plan["unchecked_m"], 3)
        notes["note"] = (f"先查碰撞到接触点前 {plan['unchecked_m'] * 100:.0f}cm,"
                         "最后这段只放过物体附近的点云(会接触物体),桌面等其他点云和自碰撞照查")
    if plan.get("direction_notes"):
        notes["direction_notes"] = plan["direction_notes"]     # 没用上的接近方向为什么没用上
    if plan.get("tool_axis_miss_deg") is not None:
        notes["tool_axis_miss_deg"] = plan["tool_axis_miss_deg"]
    if plan.get("retreat_m") is not None:
        notes["retreat_m"] = plan["retreat_m"]
        notes["tilt_fraction"] = plan["tilt_fraction"]
    return notes


def _roll_arg(args):
    """wrist_roll_deg:手腕绕工具轴在“不自转”基础上再转多少度,-180~180,不给就是 0。返回 (数, 错误)。"""
    roll = args.get("wrist_roll_deg", 0)
    if not _is_number(roll) or not -180 <= roll <= 180:
        return 0.0, "wrist_roll_deg 必须是 -180 到 180 之间的数字"
    return float(roll), None


def _number_list(value, n, name):
    """检查 value 是不是 n 个数字的列表。是 → 返回 (列表, None);不是 → 返回 (None, 错误)。"""
    if not isinstance(value, list) or len(value) != n or not all(_is_number(v) for v in value):
        return None, f"{name} 必须是 {n} 个数字的列表"
    return [float(v) for v in value], None


class Gateway:
    def __init__(self, backend, motion_cfg, confirm=None, plan_only=False, poses=None, photo_cfg=None):
        self.backend = backend
        self.cfg = motion_cfg
        self.confirm = confirm          # confirm(这次要做什么) → True 执行 / False 取消;None 表示不过问(测试用)
        self.plan_only = plan_only      # True:只规划,绝不发送运动
        self.poses = poses or {}        # configs/poses.yaml:{位姿名: {"groups": [...], "joints": {...}}}
        self.photo_cfg = photo_cfg or {"dir": "photos", "default_camera": "head_wide", "cameras": {}}
        self.objects = {}               # find_objects 找到的物体:{编号: {"xyz_m", "time", "label"}};坐标只存在这里
        self.moves = 0

    def execute(self, name, args):
        handlers = {"get_robot_state": self._robot_state,
                    "approach_object": self._approach,
                    "move_arm_end": self._move_arm_end,
                    "move_joints": self._move_joints,
                    "go_to_pose": self._go_to_pose,
                    "find_objects": self._find_objects,
                    "look_at_object": self._look_at_object,
                    "point_at_object": self._point_at_object,
                    "touch_object": self._touch_object,
                    "take_photo": self._take_photo,
                    "gripper": self._gripper,
                    "wait": self._wait}
        if name not in handlers:
            return {"ok": False, "error": f"没有工具 '{name}'"}
        try:
            return handlers[name](args)
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    # ================= 只读 =================
    def _find_objects(self, args):
        """SAM 3 按英文短语找物体。坐标存进 self.objects,模型以后只用编号指代它们。"""
        query = args.get("query")
        if not isinstance(query, str) or not query.strip():
            return {"ok": False, "error": "query 必须是非空的英文名词短语,例如 phone"}
        query = query.strip()
        if not query.isascii():
            return {"ok": False, "error": "query 要用英文名词短语(SAM 3 只认英文),例如 手机 → phone"}
        result = self.backend.find_objects(query)
        if not result.get("ok"):
            return result
        prefix = slug(query) + "_"
        for key in list(self.objects):                 # 同一个短语重新找:旧编号作废,免得指到旧位置
            if key.startswith(prefix):
                del self.objects[key]
        now = time.monotonic()
        shoulders = {arm: self.backend.shoulder_xyz(arm) for arm in ARMS}
        for obj in result["objects"]:
            self.objects[obj["id"]] = {"xyz_m": list(obj["xyz_m"]), "time": now, "label": query}
            reach = {arm: round(math.dist(obj["xyz_m"], shoulders[arm]), 2) for arm in ARMS}
            obj["shoulder_distance_m"] = reach
            obj["reachable_by"] = [arm for arm in ARMS if reach[arm] <= self.cfg["arm_reach_m"]]
        if not result["objects"]:
            return {**result, "ok": False, "error": f"画面里没有找到 {query}"}
        return {**result, "count": len(result["objects"]), "order": "按离相机由近到远,第一个最近",
                "reach_note": f"reachable_by:按现在的腰姿态,离这只手的肩 ≤ {self.cfg['arm_reach_m']}m 的才可能够得着。"
                              f"空也可以直接调 approach_object / touch_object:够不着时程序会自动前倾"
                              f"(≤{self.cfg['body_assist']['max_lean_deg']}°)/ 转腰帮忙;机器人没有底盘移动工具,不要建议走过去"}

    def _object(self, object_id):
        """按编号取 find_objects 找到的物体。返回 (物体, None) 或 (None, 错误结果)。"""
        obj = self.objects.get(object_id)
        if obj is None:
            known = list(self.objects) or "还没有,先调用 find_objects"
            return None, {"ok": False, "error": f"没有物体 '{object_id}',已找到的:{known}"}
        age = time.monotonic() - obj["time"]
        if age > self.cfg["detection_max_age_s"]:
            return None, {"ok": False, "error": f"{object_id} 的位置已过期({age:.0f}s),重新调用 find_objects"}
        return obj, None

    def _look_at_object(self, args):
        """转头,让 find_objects 找到的某个物体在画面中央;给 object_ids 时对准这几个物体的中间(拍一群人)。只动头部 3 个关节。"""
        ids = args.get("object_ids") or [args.get("object_id")]
        found = []
        for one in ids:
            item, error = self._object(one)
            if error:
                return error
            found.append(item)
        object_id = "+".join(ids)
        center = [sum(item["xyz_m"][k] for item in found) / len(found) for k in range(3)]
        obj = {"xyz_m": center, "time": min(item["time"] for item in found), "label": found[0]["label"]}
        refused = self._no_moves_left()
        if refused:
            return refused

        aim = self.backend.head_angles_to(obj["xyz_m"])
        if not aim.get("ok"):
            return aim
        current = self.backend.joint_state()
        limit = self.cfg["max_joint_delta_deg"]["head"]
        for name, value in aim["target"].items():
            delta = math.degrees(value - current[name])
            if abs(delta) > limit:
                return {"ok": False, "error": f"{name} 要转 {delta:.0f}°,超过单次上限 {limit}°"}
        if all(abs(aim["target"][j] - current[j]) < math.radians(0.5) for j in aim["target"]):
            return {"ok": True, "executed": False, "object_id": object_id,
                    "note": "物体已经在画面中央附近(差不到 0.5°),不需要转头"}

        result = self._plan_and_run_joints(aim["target"], ["head"], f"转头看 {object_id}")
        result.update(object_id=object_id, object_xyz_m=_round(obj["xyz_m"]),
                      expected_offset_deg=aim["residual_deg"], head_at_limit=aim["at_limit"])
        if aim["at_limit"]:
            result["note"] = "头已转到关节限位,物体可能无法完全居中"
        if result.get("executed") and len(found) == 1:
            result["check_after"] = self._check_centered(obj)
        return result

    def _point_at_object(self, args):
        """夹爪(工具轴)指向 find_objects 找到的物体:末端位置不动,只转方向;转不到就往身体方向后撤着转。"""
        cfg = self.cfg
        arm = args.get("arm", "right")
        if arm not in ARMS:
            return {"ok": False, "error": f"arm 只能是 {list(ARMS)}"}
        object_id = args.get("object_id")
        obj, error = self._object(object_id)
        if error:
            return error
        refused = self._no_moves_left()
        if refused:
            return refused

        start = self.backend.end_pose(arm)["xyz_m"]
        gap = math.dist(start, obj["xyz_m"])
        if gap < cfg["point_min_distance_m"]:
            return {"ok": False, "error": f"末端离 {object_id} 只有 {gap:.2f}m,太近了(至少 {cfg['point_min_distance_m']}m),"
                                          "先把手臂移开再指"}
        direction = [(o - s) / gap for o, s in zip(obj["xyz_m"], start)]
        plan = self.backend.plan_arm_to(arm, start, tool_axis=direction, retreat=cfg["tool_axis_retreat"])
        target = list(plan.get("final_target") or start)
        info = {"arm": arm, "object_id": object_id, "object_xyz_m": _round(obj["xyz_m"]),
                "start_xyz_m": _round(start), "target_xyz_m": _round(target),
                "plan": plan["summary"] if plan["ok"] else None}
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}
        error = self._check_workspace(arm, target)
        if error:
            return {"ok": False, "executed": False, "error": f"后撤后的终点超出范围:{error}", **info}
        info.update(_plan_notes(plan))

        result = self._confirm_and_run(plan, f"{ARM_NAME[arm]}夹爪指向 {object_id},{plan['summary']}", info)
        if result.get("executed"):
            end = self.backend.end_pose(arm)
            axis, _ = _tool_axis_report(end["rpy_deg"])
            to_obj = [o - e for o, e in zip(obj["xyz_m"], end["xyz_m"])]
            n = math.hypot(*to_obj)
            cos = sum(a * b for a, b in zip(axis, to_obj)) / n
            result.update(end_xyz_m=_round(end["xyz_m"]), tool_axis=axis,
                          pointing_error_deg=round(math.degrees(math.acos(max(-1.0, min(1.0, cos)))), 1),
                          distance_to_object_m=round(n, 3))
        return result

    def _touch_object(self, args):
        """夹爪尖碰到 find_objects 找到的物体(击掌、碰拳、抓取时下探),或停在它前面 gap_cm。
        唯一会接触物体的工具:物体本身在点云里,backend.plan_touch 固定两段——
        查碰撞到接触点前几厘米,再慢速直线贴上去(只放过物体附近的点云,桌面和自碰撞照查)。"""
        cfg = self.cfg
        arm = args.get("arm", "right")
        if arm not in ARMS:
            return {"ok": False, "error": f"arm 只能是 {list(ARMS)}"}
        object_id = args.get("object_id")
        obj, error = self._object(object_id)
        if error:
            return error
        gap = args.get("gap_cm", cfg["touch"]["default_gap_cm"])
        if not _is_number(gap) or not 0 <= gap <= cfg["touch"]["max_gap_cm"]:
            return {"ok": False, "error": f"gap_cm 要在 0 到 {cfg['touch']['max_gap_cm']} 之间"}
        refused = self._no_moves_left()
        if refused:
            return refused

        start = self.backend.end_pose(arm)["xyz_m"]
        dist = math.dist(start, obj["xyz_m"])
        gap_m = float(gap) / 100
        if dist <= gap_m + 0.005:
            return {"ok": True, "executed": False, "note": f"夹爪尖已经在 {object_id} 跟前,不用再动"}
        direction = [(o - s) / dist for o, s in zip(obj["xyz_m"], start)]     # 末端 → 物体
        contact = [o - gap_m * d for o, d in zip(obj["xyz_m"], direction)]
        error = self._check_workspace(arm, contact)
        if error:
            return {"ok": False, "error": error}
        plan = self.backend.plan_touch(arm, obj["xyz_m"], gap_m, direction)
        info = {"arm": arm, "object_id": object_id, "object_xyz_m": _round(obj["xyz_m"]),
                "start_xyz_m": _round(start), "tip_gap_cm": float(gap),
                "plan": plan["summary"] if plan["ok"] else None}
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}
        info.update(_plan_notes(plan))
        result = self._confirm_and_run(plan, f"{ARM_NAME[arm]}夹爪碰 {object_id},{plan['summary']}", info)
        if result.get("executed"):
            end = self.backend.end_pose(arm)["xyz_m"]
            result.update(end_xyz_m=_round(end), tip_to_object_m=round(math.dist(end, obj["xyz_m"]), 3))
        return result

    def _check_centered(self, obj):
        """转头之后再分割一次,看物体离画面中心多少像素。这次的画面也存进日志(文件夹名带 _after)。"""
        after = self.backend.find_objects(obj["label"], suffix="_after")
        if not after.get("ok"):
            return {"ok": False, "error": after.get("error")}
        if not after["objects"]:
            return {"ok": False, "error": f"转头后画面里没找到 {obj['label']}", "log_dir": after.get("log_dir")}
        same, gap = None, None                          # 转头后编号可能变,按 3D 位置找回同一个物体
        for candidate in after["objects"]:
            d = math.dist(candidate["xyz_m"], obj["xyz_m"])
            if gap is None or d < gap:
                same, gap = candidate, d
        if gap > self.cfg["look_at_match_m"]:
            return {"ok": False, "error": "转头后没认出同一个物体(位置差太多)", "log_dir": after.get("log_dir")}
        return {"ok": True, "offset_from_center_px": same["offset_from_center_px"], "log_dir": after.get("log_dir")}

    # ================= 拍照 =================
    def _take_photo(self, args):
        """存一张相机照片。不算运动,只规划模式下也照常拍。"""
        camera = args.get("camera", self.photo_cfg["default_camera"])
        if self.photo_cfg["cameras"] and camera not in self.photo_cfg["cameras"]:
            return {"ok": False, "error": f"camera 只能是 {list(self.photo_cfg['cameras'])}"}
        return self.backend.take_photo(camera, self.photo_cfg["dir"])

    # ================= 夹爪、等待 =================
    def _gripper(self, args):
        """开 / 合夹爪。算一次运动:只规划模式不动;和其他运动一样走 self.confirm(自动执行时只打印)。"""
        arm, action = args.get("arm"), args.get("action")
        if arm not in ARMS:
            return {"ok": False, "error": f"arm 只能是 {list(ARMS)}"}
        if action not in ("open", "close"):
            return {"ok": False, "error": "action 只能是 open(打开)或 close(闭合)"}
        refused = self._no_moves_left()
        if refused:
            return refused
        info = {"arm": arm, "action": action}
        if action == "open":
            what = f"{HAND_NAME[arm]}夹爪打开"
        else:
            what = f"{HAND_NAME[arm]}夹爪闭合"
        if self.plan_only:
            return {"ok": True, "executed": False, "note": f"只规划模式,{what}没有执行", **info}
        if self.confirm and not self.confirm(what):
            return {"ok": False, "executed": False, "error": "用户拒绝了这次夹爪动作", **info}
        self.moves += 1
        result = self.backend.gripper_command(arm, action)
        return {**result, "executed": bool(result.get("ok")), **info}

    def _wait(self, args):
        """等几秒再继续(“打开夹爪,过 5 秒再闭合”)。不动机器人。"""
        seconds = args.get("seconds")
        limit = self.cfg["max_wait_s"]
        if not _is_number(seconds) or not 0 < seconds <= limit:
            return {"ok": False, "error": f"seconds 要是 0 到 {limit} 之间的数字"}
        time.sleep(float(seconds))
        return {"ok": True, "waited_s": float(seconds)}

    def _robot_state(self, _args):
        joints = self.backend.joint_state()
        groups = {g: [round(math.degrees(joints[j]), 2) for j in names] for g, (_, names) in GROUPS.items()}
        ends = {}
        for arm in ARMS:
            pose = self.backend.end_pose(arm)
            axis, tilt = _tool_axis_report(pose["rpy_deg"])
            ends[arm] = {"link": pose.get("link", f"arm_{arm[0]}_end_link"), "xyz_m": _round(pose["xyz_m"]),
                         "rpy_deg": [round(float(v), 1) for v in pose["rpy_deg"]],
                         "tool_axis": axis, "tool_tilt_from_down_deg": tilt}
        grippers = {arm: self.backend.gripper_state(arm) for arm in ARMS}
        return {"ok": True, "frame": "base_link", "joints_deg": groups, "ends": ends, "grippers": grippers,
                "moves_left": self.cfg["max_moves_per_command"] - self.moves}

    # ================= 运动:共用的部分 =================
    def _no_moves_left(self):
        if self.moves >= self.cfg["max_moves_per_command"]:
            return {"ok": False, "executed": False, "error": "这条指令的运动次数已用完,请用户重新下指令"}
        return None

    def _confirm_and_run(self, plan, prompt, info):
        """规划和检查都通过之后:只规划模式到此为止;否则问人,按 y 才执行。"""
        if self.plan_only:
            return {"ok": True, "executed": False, "note": "只规划模式,机器人没有动", **info}
        if self.confirm and not self.confirm(prompt):
            return {"ok": False, "executed": False, "error": "用户拒绝了这次运动", **info}
        self.moves += 1
        done = self.backend.execute(plan)
        result = {"ok": done["ok"], "executed": done["ok"], "msg": done["msg"], **info}
        if "contact" in done:                    # touch_object 最后一段的六维力结果
            result["contact"] = done["contact"]
        return result

    def _check_workspace(self, arm, target):
        for axis, value in zip("xyz", target):
            low, high = self.cfg["workspace"][arm][axis]
            if not low <= value <= high:
                return f"目标 {axis}={value:.3f}m 超出{ARM_NAME[arm]}工作范围 [{low}, {high}]"
        return None

    # ================= approach_object:末端到物体正上方 =================
    def _approach(self, args):
        cfg = self.cfg
        arm = args.get("arm", "right")
        if arm not in ARMS:
            return {"ok": False, "error": f"arm 只能是 {list(ARMS)}"}
        obj, error = self._object(args.get("object_id"))
        if error:
            return error
        object_xyz = obj["xyz_m"]
        refused = self._no_moves_left()
        if refused:
            return refused

        standoff = args.get("standoff_cm", cfg["standoff_default_m"] * 100)
        if not _is_number(standoff):
            return {"ok": False, "error": "standoff_cm 必须是数字"}
        standoff = float(standoff) / 100
        if not cfg["standoff_min_m"] <= standoff <= cfg["standoff_max_m"]:
            return {"ok": False, "error": f"standoff_cm 必须在 {cfg['standoff_min_m'] * 100:.0f} 到 "
                                          f"{cfg['standoff_max_m'] * 100:.0f} 之间"}

        target = [object_xyz[0], object_xyz[1], object_xyz[2] + standoff]
        error = self._check_workspace(arm, target)
        if error:
            return {"ok": False, "error": error}

        start = self.backend.end_pose(arm)["xyz_m"]
        travel = math.dist(start, target)
        if travel > cfg["max_travel_m"]:
            return {"ok": False, "error": f"移动距离 {travel:.2f}m 超过上限 {cfg['max_travel_m']}m"}

        roll, error = _roll_arg(args)
        if error:
            return {"ok": False, "error": error}
        axis, error = _tool_axis_arg(args)
        if error:
            return {"ok": False, "error": error}
        plan = self.backend.plan_arm_to(arm, target, orientation=cfg["goal_orientation"]["approach_object"],
                                        wrist_roll_deg=roll, tool_axis=axis)
        info = {"arm": arm, "target_xyz_m": _round(target), "standoff_m": round(standoff, 3),
                "travel_m": round(travel, 3), "plan": plan["summary"] if plan["ok"] else None}
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}
        info.update(_plan_notes(plan))
        floor = object_xyz[2] + cfg["min_path_above_object_m"]
        if plan["lowest_z_m"] < floor:
            return {"ok": False, "executed": False,
                    "error": f"路径最低点 {plan['lowest_z_m']:.2f}m 低于 {floor:.2f}m,可能穿过桌面。"
                             f"请用户先把{ARM_NAME[arm]}移到桌面上方。", **info}

        result = self._confirm_and_run(
            plan, f"{ARM_NAME[arm]}末端将移动到 {_round(target)}(直线距离 {travel * 100:.1f}cm,"
                  f"物体上方 {standoff * 100:.0f}cm),{plan['summary']}", info)
        if result.get("executed"):
            end = self.backend.end_pose(arm)["xyz_m"]
            result.update(end_xyz_m=_round(end), error_to_target_mm=round(math.dist(end, target) * 1000, 1),
                          distance_to_object_m=round(math.dist(end, object_xyz), 3))
        return result

    # ================= move_arm_end:末端相对移动 / 转动 =================
    def _move_arm_end(self, args):
        cfg = self.cfg
        arm = args.get("arm")
        if arm not in ARMS:
            return {"ok": False, "error": f"arm 只能是 {list(ARMS)}"}
        refused = self._no_moves_left()
        if refused:
            return refused

        offset_cm, error = _number_list(args.get("offset_cm", [0, 0, 0]), 3, "offset_cm")
        if error:
            return {"ok": False, "error": error}
        rotate_deg, error = _number_list(args.get("rotate_deg", [0, 0, 0]), 3, "rotate_deg")
        if error:
            return {"ok": False, "error": error}
        if not any(offset_cm) and not any(rotate_deg) and not args.get("wrist_roll_deg") and not args.get("tool_axis"):
            return {"ok": False, "error": "offset_cm、rotate_deg、tool_axis、wrist_roll_deg 都没给,不需要运动"}

        offset_m = [v / 100 for v in offset_cm]
        distance = math.hypot(*offset_m)
        if distance > cfg["max_offset_m"]:
            return {"ok": False, "error": f"一次移动 {distance:.2f}m 超过上限 {cfg['max_offset_m']}m,请分几次移动"}

        now = self.backend.end_pose(arm)
        start, start_rpy = now["xyz_m"], now["rpy_deg"]
        target = [s + d for s, d in zip(start, offset_m)]
        error = self._check_workspace(arm, target)
        if error:
            return {"ok": False, "error": error}

        target_rpy = None                                   # None = 保持当前姿态
        turn = 0.0
        if any(rotate_deg):
            target_rpy = rotate_rpy_deg(start_rpy, rotate_deg)
            turn = rotation_between_deg(start_rpy, target_rpy)
            if turn > cfg["max_rotate_deg"]:
                return {"ok": False, "error": f"一次转动 {turn:.0f}° 超过上限 {cfg['max_rotate_deg']}°,请分几次转"}

        roll, error = _roll_arg(args)
        if error:
            return {"ok": False, "error": error}
        axis, error = _tool_axis_arg(args)
        if error:
            return {"ok": False, "error": error}
        if target_rpy is not None and (roll or axis is not None):
            return {"ok": False, "error": "rotate_deg 不能和 tool_axis / wrist_roll_deg 一起用:"
                                          "让夹爪朝某个方向用 tool_axis,手腕自转用 wrist_roll_deg"}
        plan = self.backend.plan_arm_to(arm, target, target_rpy, orientation=cfg["goal_orientation"]["move_arm_end"],
                                        wrist_roll_deg=roll, tool_axis=axis,
                                        retreat=None if any(offset_cm) else cfg["tool_axis_retreat"])
        if plan["ok"] and plan.get("final_target") is not None:
            target = list(plan["final_target"])            # 原地转不到,后端自己往后撤了:按实际终点再查一次范围
            error = self._check_workspace(arm, target)
            if error:
                return {"ok": False, "executed": False, "error": f"后撤后的终点超出范围:{error}"}
        info = {"arm": arm, "start_xyz_m": _round(start), "target_xyz_m": _round(target),
                "target_rpy_deg": target_rpy, "requested_rotate_deg": round(turn, 1),
                "plan": plan["summary"] if plan["ok"] else None}
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}
        info.update(_plan_notes(plan))
        dip = cfg.get("max_path_dip_m")                     # None = 不检查,碰撞完全交给规划器
        floor = None if dip is None else min(start[2], target[2]) - dip
        if floor is not None and plan["lowest_z_m"] < floor:
            return {"ok": False, "executed": False,
                    "error": f"规划出的路径最低点 {plan['lowest_z_m']:.2f}m 比起点和终点低太多(下限 {floor:.2f}m),"
                             "可能扫到桌面,已拒绝", **info}

        what = f"偏移 {[round(v, 1) for v in offset_cm]}cm" + (f",转 {turn:.0f}°" if turn else "")
        result = self._confirm_and_run(plan, f"{ARM_NAME[arm]}末端 {_round(start)} → {_round(target)}({what}),"
                                             f"{plan['summary']}", info)
        if result.get("executed"):
            end = self.backend.end_pose(arm)
            axis, tilt = _tool_axis_report(end["rpy_deg"])
            result.update(end_xyz_m=_round(end["xyz_m"]), end_rpy_deg=end["rpy_deg"],
                          tool_axis=axis, tool_tilt_from_down_deg=tilt,
                          actual_turn_deg=round(rotation_between_deg(start_rpy, end["rpy_deg"]), 1),
                          error_to_target_mm=round(math.dist(end["xyz_m"], target) * 1000, 1))
        return result

    # ================= move_joints:某一组关节转动 =================
    def _move_joints(self, args):
        group = args.get("group")
        if group not in GROUPS:
            return {"ok": False, "error": f"group 只能是 {list(GROUPS)}"}
        refused = self._no_moves_left()
        if refused:
            return refused

        names = GROUPS[group][1]
        current = self.backend.joint_state()
        has_deltas, has_angles = "deltas_deg" in args, "angles_deg" in args
        if has_deltas == has_angles:
            return {"ok": False, "error": "deltas_deg(相对转多少)和 angles_deg(转到多少)必须给且只给一个"}
        if has_deltas:
            values, error = _number_list(args["deltas_deg"], len(names), "deltas_deg")
            if error:
                return {"ok": False, "error": error}
            target = {j: current[j] + math.radians(d) for j, d in zip(names, values)}
        else:
            values, error = _number_list(args["angles_deg"], len(names), "angles_deg")
            if error:
                return {"ok": False, "error": error}
            target = {j: math.radians(a) for j, a in zip(names, values)}

        limit = self.cfg["max_joint_delta_deg"][group]
        for i, j in enumerate(names, start=1):
            delta = math.degrees(target[j] - current[j])
            if abs(delta) > limit:
                return {"ok": False, "error": f"{GROUP_NAME[group]}第 {i} 关节要转 {delta:.0f}°,"
                                              f"超过单次上限 {limit}°,请分几次转"}
        if all(abs(target[j] - current[j]) < 1e-4 for j in names):
            return {"ok": False, "error": "目标和当前关节角相同,不需要运动"}
        return self._plan_and_run_joints(target, [group], f"{GROUP_NAME[group]}关节")

    # ================= go_to_pose:去录好的位姿 =================
    def _go_to_pose(self, args):
        name = args.get("name")
        if name not in self.poses:
            return {"ok": False, "error": f"没有位姿 '{name}',可选:{list(self.poses)}"}
        refused = self._no_moves_left()
        if refused:
            return refused
        pose = self.poses[name]
        groups = args.get("groups") or list(pose["groups"])
        if not isinstance(groups, list) or any(g not in pose["groups"] for g in groups):
            return {"ok": False, "error": f"位姿 {name} 只包含这些组:{pose['groups']}"}
        target = {j: float(pose["joints"][j]) for g in groups for j in GROUPS[g][1]}
        return self._plan_and_run_joints(target, groups, f"位姿 {name}")

    def _plan_and_run_joints(self, target, groups, label):
        """move_joints 和 go_to_pose 共用:限位检查 → 规划(碰撞检查 + 绕行) → 确认 → 执行。"""
        limits = self.backend.joint_limits()
        current = self.backend.joint_state()
        clamped = []
        for j, v in list(target.items()):
            if j not in limits:
                continue
            low, high = limits[j][0] + LIMIT_MARGIN_RAD, limits[j][1] - LIMIT_MARGIN_RAD   # 真正能用的范围
            if not low <= v <= high:                  # 超出:停在能用范围的边上(“抬头到底”一次就做完)
                target[j] = min(max(v, low), high)
                clamped.append(f"{j} 目标 {math.degrees(v):.1f}° 超出可用范围 "
                               f"[{math.degrees(low):.1f}°, {math.degrees(high):.1f}°],改为 {math.degrees(target[j]):.1f}°")
        if all(abs(target[j] - current[j]) < math.radians(0.3) for j in target):
            if clamped:
                return {"ok": False, "error": "已经在关节限位边上,不能再往这个方向转了:" + ";".join(clamped)}
            return {"ok": False, "error": "目标和当前关节角相同,不需要运动"}

        speed = min(self.cfg["joint_speed_rad_s"][g] for g in groups)
        changes = {g: round(max(abs(math.degrees(target[j] - current[j])) for j in GROUPS[g][1]), 1)
                   for g in groups}
        plan = self.backend.plan_joints(target, speed)
        info = {"groups": groups, "max_change_deg": changes, "plan": plan["summary"] if plan["ok"] else None}
        if clamped:
            info["clamped_to_limit"] = clamped
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}

        text = ",".join(f"{GROUP_NAME[g]}最大 {d}°" for g, d in changes.items())
        result = self._confirm_and_run(plan, f"{label}:{text},峰值速度 {speed} rad/s,{plan['summary']}", info)
        if result.get("executed"):
            now = self.backend.joint_state()
            result["joints_deg"] = {g: [round(math.degrees(now[j]), 2) for j in GROUPS[g][1]] for g in groups}
        return result
