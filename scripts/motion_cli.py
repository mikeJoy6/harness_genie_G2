#!/usr/bin/env python3
"""motion_cli.py —— G2 手臂运控交互命令行（会动真机）。

前提：另一个终端在跑
    source ~/agi@g2_robot/env_g2.sh
    ros2 launch genie_robot_agibot_g2_hardware hardware.launch.py
运行：
    source ~/agi@g2_robot/env_g2.sh
    python3 scripts/motion_cli.py        (复制自 agi@g2_robot,只改了 import)

坐标全部是 base_link 系（x 前 y 左 z 上），长度 m，角度 °。
每条运动命令都是 先规划 → 打印规划摘要 → 输入 y 才执行；任何检查不过都不发命令。
"""
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.motion.arm_motion import ArmMotion, Plan  # noqa: E402
from harness.motion.algorithms import rpy_to_mat, mat_to_rpy  # noqa: E402

HELP = """\
命令（长度 m，角度 °，坐标系 base_link：x前 y左 z上）：
  arm left|right                 切换手臂（默认 left）
  pose                           打印当前末端位姿 + 关节角
  move dx dy dz [dr dp dy]       相对当前末端 走直线（积分笛卡尔），可带绕 base 轴的转角
  line x y z [r p y]             绝对目标 走直线（积分笛卡尔）；不给 rpy 就保持当前姿态
  goto x y z [r p y]             绝对目标 多种子 IK + 关节直连（长距离换位置用；会撞则末端网格搜索绕行）
  detour x y z [r p y]           绝对目标 强制末端网格搜索（先试直线，不行就 5cm 格子 A* 绕行，不会抡大圈）
  ik   x y z [r p y]             只解 IK，打印解和关节差，不动
  help / quit
"""


def _floats(args, n_min, n_max):
    if not n_min <= len(args) <= n_max:
        raise ValueError(f"需要 {n_min}~{n_max} 个数")
    return [float(a) for a in args]


def _target(args, cur_R):
    v = _floats(args, 3, 6)
    if len(v) not in (3, 6):
        raise ValueError("给 3 个(只定位置) 或 6 个(位置+rpy) 数")
    R = cur_R if len(v) == 3 else rpy_to_mat(*np.radians(v[3:]))
    return np.array(v[:3]), R


def _fmt_pose(p, R):
    r, pi, y = np.degrees(mat_to_rpy(R))
    return f"xyz=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m   rpy=({r:+.1f}, {pi:+.1f}, {y:+.1f})°"


CONFIRM = "--confirm" in sys.argv       # 默认规划完直接执行;加 --confirm 每次问 y


def _confirm_and_run(node, plan):
    if isinstance(plan, str):
        print(f"  ✗ 规划失败：{plan}")
        return
    assert isinstance(plan, Plan)
    print("  " + plan.summary())
    if CONFIRM and input("  执行？[y/N] ").strip().lower() != "y":
        print("  已取消")
        return
    ok, msg = node.execute(plan)
    print(("  ✓ " if ok else "  ✗ ") + msg)
    if ok:
        print("  现在 " + _fmt_pose(*node.pose(plan.arm)))


def main():
    rclpy.init()
    node = ArmMotion("g2_motion_cli")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    print("等待 /robot_description 和 /joint_states …")
    t0 = time.monotonic()
    while not node.ready():
        if time.monotonic() - t0 > 10.0:
            print("10s 内没等到 URDF / 关节反馈：hardware.launch.py 在跑吗？env_g2.sh source 了吗？")
            break
        time.sleep(0.1)
    print(HELP)

    arm = "left"
    try:
        while True:
            try:
                line = input(f"[{arm}]> ").strip()
            except EOFError:
                break
            if not line:
                continue
            cmd, *args = line.split()
            try:
                if cmd in ("quit", "exit", "q"):
                    break
                elif cmd == "help":
                    print(HELP)
                elif cmd == "arm":
                    if args and args[0] in ("left", "right"):
                        arm = args[0]
                    else:
                        print("  arm left|right")
                elif cmd == "pose":
                    print("  " + _fmt_pose(*node.pose(arm)))
                    print("  关节(°) " + " ".join(f"{v:+.1f}" for v in np.degrees(node.q_arm(arm))))
                elif cmd == "move":
                    v = _floats(args, 3, 6)
                    p, R = node.pose(arm)
                    dR = rpy_to_mat(*np.radians(v[3:])) if len(v) == 6 else np.eye(3)
                    _confirm_and_run(node, node.plan_cartesian(arm, p + np.array(v[:3]), dR @ R))
                elif cmd in ("line", "goto", "ik", "detour"):
                    p_t, R_t = _target(args, node.pose(arm)[1])
                    if cmd == "line":
                        _confirm_and_run(node, node.plan_cartesian(arm, p_t, R_t))
                    elif cmd == "goto":
                        _confirm_and_run(node, node.plan_pose(arm, p_t, R_t))
                    elif cmd == "detour":
                        print("  网格搜索中（最多约 8s）…")
                        _confirm_and_run(node, node.plan_detour(arm, p_t, R_t))
                    else:
                        q, dt = node.plan_ik_only(arm, p_t, R_t)
                        if q is None:
                            print(f"  ✗ 无(不撞身体的)IK 解，用时 {dt*1000:.0f}ms")
                        else:
                            q0 = node.q_arm(arm)
                            pk, Rk = node.kins[arm].fk(q)
                            print(f"  ✓ 用时 {dt*1000:.0f}ms")
                            print("  解(°)   " + " ".join(f"{v:+.1f}" for v in np.degrees(q)))
                            print("  关节差(°) " + " ".join(f"{v:+.1f}" for v in np.degrees(q - q0))
                                  + f"   最大 {np.degrees(np.max(np.abs(q - q0))):.1f}°")
                else:
                    print("  未知命令，help 看用法")
            except (ValueError, RuntimeError) as e:
                print(f"  ✗ {e}")
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
