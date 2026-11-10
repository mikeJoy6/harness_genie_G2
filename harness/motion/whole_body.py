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


def _wait(fut, timeout):
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.005)
    return fut.result() if fut.done() else None


class WholeBody:
    def __init__(self, node, cb_group=None):
        self.node = node
        self.lock = threading.Lock()
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
    def _valid(self, joints):
        req = GetStateValidity.Request()
        req.robot_state.joint_state.name = list(joints)
        req.robot_state.joint_state.position = [float(v) for v in joints.values()]
        req.robot_state.is_diff = True             # 保留附着物（纸板）
        res = _wait(self.sv.call_async(req), 1.0)
        if res is None:
            return False, "碰撞检查超时"
        if not res.valid:
            pairs = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts})
            return False, "撞：" + ", ".join(f"{a}↔{b}" for a, b in pairs[:4])
        return True, ""

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
        返回 (成功?, 说明)。"""
        say = progress or (lambda _m: None)
        q0 = self.snapshot()
        missing = [j for j in target if j not in q0]
        if missing:
            return False, f"/joint_states 里没有 {missing[:3]}"
        groups = [g for g, (_, js) in GROUPS.items() if any(j in target for j in js)]
        for g in groups:                                  # 一组只给了一部分关节：其余保持当前
            for j in GROUPS[g][1]:
                target.setdefault(j, q0[j])
        dmax = max(abs(target[j] - q0[j]) for j in target)
        if dmax < 1e-3:
            return True, "已经在目标位姿"
        if check:
            if not self.sv.wait_for_service(timeout_sec=WAIT):
                return False, "/check_state_validity 不在（move_group 没起？）；确认安全可加 check=0 跳过"
            say(f"碰撞检查（最大关节差 {math.degrees(dmax):.1f}°）")
            ok, why = self.check_path(q0, target, check_step)
            if not ok:
                if detour is None:
                    return False, why
                say(f"直线{why}，保底绕行规划…")
                path, note = detour.plan(q0, dict(target))
                if path is None:
                    return False, f"直线{why}；保底绕行：{note}"
                say(f"保底绕行：{note}，{len(path)} 段")
                via = f"直线{why} → 保底绕行 {note}；"
            else:
                path, via = [target], ""
        else:
            path, via = [target], ""
        qs = [q0] + [dict(q0, **w) for w in path]
        Ts = [max(min_duration, 1.5 * max(abs(b[j] - a[j]) for j in target) / speed)    # 三次曲线峰值速度 = 1.5·Δ/T
              for a, b in zip(qs[:-1], qs[1:])]
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
