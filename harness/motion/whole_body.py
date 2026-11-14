"""whole_body.py —— 全身关节点到点（腰 / 头 / 双臂一起走），任务层用的运控。

  · 关节空间直线插值，时间按平滑三次曲线（起止速度 0）；最慢的关节峰值速度 = speed，至少 min_duration 秒。
  · 发之前沿途每隔 check_step 弧度问一次 MoveIt /check_state_validity（整机 + 点云 + 附着的纸板，is_diff=True），
    撞了就不发。move_group 没起时默认不动（check=False 才放行）。
  · 直线会撞且给了 detour（joint_detour.JointDetour）：保底绕行——经录好位姿中转 / 关节 RRT，得到折线，
    每个拐点停一下（每段同样三次曲线），一条轨迹发出去。
  · 4 个 JTC 同时发；执行完再按 /joint_states 实测确认到位（JTC 不查到位：机器人不跟时它照样报成功）。
  · cancel()：取消所有在执行的目标（别的线程里 goto 会返回"已取消"）。
需要 hardware.launch 在跑（JTC + move_group）。
"""
import math
import threading
import time

import numpy as np
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .joints import GROUPS

DT = 0.1                 # 轨迹点间隔 (s)
SETTLE_TOL = 0.02        # 到位容差 (rad，~1.1°)
SETTLE_TIMEOUT = 3.0     # JTC 完成后等实测到位 (s)
WAIT = 2.0               # 等 action server / 服务 (s)
ESCAPE_RAD = 0.10        # 起点就有、涉及这次要动的部件的接触(夹着鼠标回初始位姿):每个关节离起点都在这么多以内时不算,
                         # 让手先“脱离”;走出这个范围还撞就是真撞。和 arm_motion.ESCAPE_M(末端 6cm)同一个思路。


def _wait(fut, timeout):
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.005)
    return fut.result() if fut.done() else None


class JointPlan:
    """WholeBody.plan() 的结果：qs[0] 是规划时的实测起点，qs[1:] 是折线路点，Ts[k] 是第 k 段用时。"""
    def __init__(self, groups, target, qs, Ts, note):
        self.groups, self.target, self.qs, self.Ts, self.note = groups, target, qs, Ts, note
        self.duration = sum(Ts)

    def summary(self):
        if not self.Ts:
            return self.note
        dmax = max(abs(self.target[j] - self.qs[0][j]) for j in self.target)
        return (f"{self.note}：{'+'.join(self.groups)}，{len(self.Ts)} 段，{self.duration:.1f}s，"
                f"最大关节变化 {math.degrees(dmax):.1f}°")


# 每个关节组动的时候,哪些 link 跟着动(按 G2 URDF 的 link 名前缀)。body 一动,上面的手臂、头全都跟着动。
MOVING_LINKS = {"left_arm": ("arm_l_", "tcp_l_"), "right_arm": ("arm_r_", "tcp_r_"), "head": ("head_link",)}
ROBOT_LINKS = ("arm_", "tcp_", "head_", "body_", "base_link", "chassis")


def _moves(body, groups):
    """MoveIt 碰撞结果里的一个 body(link 名 / "<octomap>" / 碰撞体名)在这次运动里会不会动。"""
    if "body" in groups:                           # 腰动:除了底座、底盘,机器人上的东西都动
        return body.startswith(ROBOT_LINKS) and body != "base_link" and not body.startswith("chassis")
    return any(body.startswith(prefix) for g in groups for prefix in MOVING_LINKS.get(g, ()))


class WholeBody:
    def __init__(self, node, cb_group=None):
        self.node = node
        self.lock = threading.Lock()
        self._allowed = set()        # plan() 期间:起点已有、这次运动不动的部件之间的接触,检查时放过
        self._escape = set()         # plan() 期间:起点已有、涉及要动的部件的接触,只在起点附近(ESCAPE_RAD)放过
        self._start = {}             # plan() 的起点关节角
        self.q = {}
        self.q_stamp = 0.0
        # joint_state_broadcaster 这里发的是 BEST_EFFORT：默认 RELIABLE 订阅 QoS 不兼容，一条都收不到
        node.create_subscription(JointState, "/joint_states", self._on_js, qos_profile_sensor_data,
                                 callback_group=cb_group)
        self.actions = {g: ActionClient(node, FollowJointTrajectory, f"/{c}/follow_joint_trajectory",
                                        callback_group=cb_group) for g, (c, _) in GROUPS.items()}
        self.sv = node.create_client(GetStateValidity, "/check_state_validity", callback_group=cb_group)
        self.active = []

    def _on_js(self, m):
        with self.lock:
            self.q.update(zip(m.name, m.position))
            self.q_stamp = time.monotonic()

    def snapshot(self):
        """当前全身关节 {名: 弧度}；/joint_states 1s 内没更新抛 RuntimeError。"""
        with self.lock:
            if time.monotonic() - self.q_stamp > 1.0:
                raise RuntimeError("/joint_states 1s 没更新（hardware.launch 在跑吗？）")
            return dict(self.q)

    # ── 碰撞检查 ─────────────────────────────────────────────
    def _contacts(self, joints):
        """问 MoveIt 这个状态撞不撞。返回 (撞的 (body1, body2) 对的列表, 错误)。不撞 = ([], None)。"""
        req = GetStateValidity.Request()
        req.robot_state.joint_state.name = list(joints)
        req.robot_state.joint_state.position = [float(v) for v in joints.values()]
        req.robot_state.is_diff = True             # 保留附着物（纸板）
        res = _wait(self.sv.call_async(req), 1.0)
        if res is None:
            return None, "碰撞检查超时"
        if res.valid:
            return [], None
        return sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts}), None

    def _valid(self, joints):
        pairs, error = self._contacts(joints)
        if error:
            return False, error
        pairs = [p for p in pairs if p not in self._allowed]      # 起点就有、且这次不动的部件之间的接触,不算
        if pairs and self._escape and self._near_start(joints):
            pairs = [p for p in pairs if p not in self._escape]   # 还在起点附近:起点就有的接触先不算(正在脱离)
        if pairs:
            return False, "撞：" + ", ".join(f"{a}↔{b}" for a, b in pairs[:4])
        return True, ""

    def _near_start(self, joints):
        for j, v in joints.items():
            if j in self._start and abs(v - self._start[j]) > ESCAPE_RAD:
                return False
        return True

    def check_path(self, q0, q1, step):
        n = max(1, int(math.ceil(max(abs(q1[j] - q0[j]) for j in q1) / step)))
        for k in range(n + 1):
            s = k / n
            ok, why = self._valid({j: q0[j] + (q1[j] - q0[j]) * s for j in q1})
            if not ok:
                return False, f"沿途 {s:.0%} 处{why}"
        return True, ""

    # ── 执行 ─────────────────────────────────────────────────
    @staticmethod
    def _traj(names, qs, Ts):
        """折线 qs[0]→qs[1]→…，第 k 段用时 Ts[k]，每段三次曲线（拐点速度 0）。"""
        tr = JointTrajectory(joint_names=list(names))
        t0 = 0.0
        for q0, q1, T in zip(qs[:-1], qs[1:], Ts):
            a, d = np.array([q0[j] for j in names]), np.array([q1[j] - q0[j] for j in names])
            n = max(2, int(math.ceil(T / DT)))
            for k in range(1, n + 1):
                t = T * k / n
                tau = t / T
                p = JointTrajectoryPoint()
                p.positions = (a + d * (3 * tau ** 2 - 2 * tau ** 3)).tolist()
                p.velocities = (d * (6 * tau - 6 * tau ** 2) / T).tolist()
                p.time_from_start = Duration(seconds=t0 + t).to_msg()
                tr.points.append(p)
            t0 += T
        return tr

    def goto(self, target, speed=0.3, min_duration=2.0, check=True, check_step=0.05, cancelled=None, progress=None,
             detour=None):
        """target：{关节名: 弧度}（可以只含部分组）。detour：直线会撞时的保底（JointDetour），None = 撞了就失败。
        返回 (成功?, 说明)。= plan() + run()，中间不停。"""
        plan = self.plan(target, speed, min_duration, check, check_step, progress, detour)
        if isinstance(plan, str):
            return False, plan
        return self.run(plan, cancelled, progress)

    def plan(self, target, speed=0.3, min_duration=2.0, check=True, check_step=0.05, progress=None, detour=None):
        """只规划不动：碰撞检查 + 必要时保底绕行。返回 JointPlan 或失败原因(str)。
        规划和执行分开，是为了让 harness 在两者之间打印规划、等人确认。"""
        say = progress or (lambda _m: None)
        self._allowed, self._escape = set(), set()
        target = dict(target)
        q0 = self.snapshot()
        missing = [j for j in target if j not in q0]
        if missing:
            return f"/joint_states 里没有 {missing[:3]}"
        groups = [g for g, (_, js) in GROUPS.items() if any(j in target for j in js)]
        for g in groups:                                  # 一组只给了一部分关节：其余保持当前
            for j in GROUPS[g][1]:
                target.setdefault(j, q0[j])
        dmax = max(abs(target[j] - q0[j]) for j in target)
        if dmax < 1e-3:
            return JointPlan(groups, target, [q0], [], "已经在目标位姿")
        if check:
            if not self.sv.wait_for_service(timeout_sec=WAIT):
                return "/check_state_validity 不在（move_group 没起？）；确认安全可加 check=0 跳过"
            # 起点就已经在接触(比如右手刚碰过人手、还挨着点云),而这次只动头:那对接触跟这次运动无关,不该拦。
            # 只放过“起点就有 + 两边这次都不动”的接触;新出现的、或者涉及要动的部件的,照样拦。
            self._allowed, self._escape = set(), set()
            start_pairs, _ = self._contacts(q0)
            for pair in start_pairs or []:
                if _moves(pair[0], groups) or _moves(pair[1], groups):
                    self._escape.add(pair)
                else:
                    self._allowed.add(pair)
            self._start = {j: q0[j] for j in target}
            if self._allowed:
                say("起点已有、和这次运动无关的接触不算：" + ", ".join(f"{a}↔{b}" for a, b in sorted(self._allowed)[:4]))
            if self._escape:
                say(f"起点就在接触、离起点 {math.degrees(ESCAPE_RAD):.0f}° 以内不算(脱离)：" +
                    ", ".join(f"{a}↔{b}" for a, b in sorted(self._escape)[:4]))
            say(f"碰撞检查（最大关节差 {math.degrees(dmax):.1f}°）")
            ok, why = self.check_path(q0, target, check_step)
            if not ok:
                if detour is None:
                    return why
                say(f"直线{why}，保底绕行规划…")
                path, note = detour.plan(q0, dict(target))
                if path is None:
                    return f"直线{why}；保底绕行：{note}"
                say(f"保底绕行：{note}，{len(path)} 段")
                via = f"直线{why} → 保底绕行 {note}"
            else:
                path, via = [target], "关节直线插值，沿途不撞"
        else:
            path, via = [target], "关节直线插值（未做碰撞检查）"
        if check and self._allowed:
            via += "（起点已有、和这次运动无关的接触不算：" + ", ".join(f"{a}↔{b}" for a, b in sorted(self._allowed)[:3]) + "）"
        if check and self._escape:
            via += "（起点就在接触，先脱离：" + ", ".join(f"{a}↔{b}" for a, b in sorted(self._escape)[:3]) + "）"
        self._allowed, self._escape = set(), set()
        qs = [q0] + [dict(q0, **w) for w in path]
        Ts = [max(min_duration, 1.5 * max(abs(b[j] - a[j]) for j in target) / speed)    # 三次曲线峰值速度 = 1.5·Δ/T
              for a, b in zip(qs[:-1], qs[1:])]
        return JointPlan(groups, target, qs, Ts, via)

    def run(self, plan, cancelled=None, progress=None):
        """执行 plan()。机器人已离开规划起点(>0.05rad)就不发，要求重新规划。返回 (成功?, 说明)。"""
        say = progress or (lambda _m: None)
        if not plan.Ts:
            return True, plan.note
        groups, target, qs, Ts = plan.groups, plan.target, plan.qs, plan.Ts
        q_now = self.snapshot()
        drift = max(abs(q_now[j] - qs[0][j]) for j in target)
        if drift > 0.05:
            return False, f"机器人已离开规划起点 {math.degrees(drift):.1f}°，请重新规划"
        via = plan.note + "；"
        T = sum(Ts)
        handles = []
        for g in groups:
            ac = self.actions[g]
            if not ac.wait_for_server(timeout_sec=WAIT):
                self._cancel(handles)
                return False, f"{GROUPS[g][0]} 不在线"
            goal = FollowJointTrajectory.Goal(trajectory=self._traj(GROUPS[g][1], qs, Ts))
            h = _wait(ac.send_goal_async(goal), WAIT)
            if h is None or not h.accepted:
                self._cancel(handles)
                return False, f"{GROUPS[g][0]} 拒绝了目标"
            handles.append((g, h, h.get_result_async()))
        self.active = handles
        say(f"执行 {'+'.join(groups)}，{T:.1f}s")
        t0 = time.monotonic()
        try:
            while not all(r.done() for _, _, r in handles):
                if cancelled is not None and cancelled.is_set():
                    self._cancel(handles)
                    return False, "已取消"
                if time.monotonic() - t0 > T + 5.0:
                    self._cancel(handles)
                    return False, f"超过 {T + 5:.1f}s 没执行完，已取消"
                time.sleep(0.02)
        finally:
            self.active = []
        for g, _, r in handles:
            res = r.result()
            if res.status != 4 or res.result.error_code != 0:          # 4 = SUCCEEDED
                return False, f"{GROUPS[g][0]} 执行失败 status={res.status} {res.result.error_string}"
        ok, msg = self._settle(target, t0)
        return ok, via + msg

    def _settle(self, target, t0):
        t1 = time.monotonic()
        while True:
            q = self.snapshot()
            j, err = max(((j, abs(q[j] - target[j])) for j in target), key=lambda x: x[1])
            if err <= SETTLE_TOL:
                return True, f"到位（{time.monotonic() - t0:.1f}s，最大误差 {math.degrees(err):.2f}°）"
            if time.monotonic() - t1 > SETTLE_TIMEOUT:
                return False, (f"JTC 报完成但实测没到位：{j} 差 {math.degrees(err):.1f}°"
                               "（机器人没跟命令？看 /gdk/joint_state）")
            time.sleep(0.05)

    @staticmethod
    def _cancel(handles):
        for _, h, _ in handles:
            h.cancel_goal_async()

    def cancel(self):
        self._cancel(self.active)
        return bool(self.active)
