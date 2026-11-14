"""把全身移到 configs/poses.yaml 里录好的位姿。只需要 hardware.launch.py 在跑。

  source ~/agi@g2_robot/env_g2.sh
  python3 scripts/goto_pose.py --list                 列出位姿
  python3 scripts/goto_pose.py look                   全身(腰、头、双臂)到 look
  python3 scripts/goto_pose.py look --groups head     只动头

沿途每 0.05rad 问一次 MoveIt 自碰撞;直线会撞就先试经其他录好位姿中转,再试关节 RRT。
执行前打印每组最大关节变化后直接执行(加 --confirm 则输入 y 才动);执行中 Ctrl-C 取消。
"""
import argparse
import math
import sys
import threading
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

POSES_PATH = ROOT / "configs/poses.yaml"


def load_poses():
    return yaml.safe_load(POSES_PATH.read_text(encoding="utf-8")) or {}


def joint_limits(urdf):
    from harness.motion import kin  # noqa: F401  导入时会屏蔽 URDF 解析器对 G2 扩展标签的告警
    from urdf_parser_py.urdf import URDF
    return {j.name: (j.limit.lower, j.limit.upper) for j in URDF.from_xml_string(urdf).joints
            if j.limit is not None and j.type != "continuous"}


def main():
    parser = argparse.ArgumentParser(description="全身移到录好的位姿")
    parser.add_argument("name", nargs="?", help="位姿名,见 --list")
    parser.add_argument("--groups", help="只动哪些组,逗号分隔:body,head,left_arm,right_arm")
    parser.add_argument("--speed", type=float, default=0.2, help="最慢关节的峰值速度 rad/s")
    parser.add_argument("--list", action="store_true", help="列出位姿")
    parser.add_argument("--confirm", action="store_true", help="执行前等人按 y(默认直接执行)")
    args = parser.parse_args()

    poses = load_poses()
    if args.list or not args.name:
        for name, pose in poses.items():
            print(f"{name:<10} {'+'.join(pose['groups'])}  {pose.get('note', '')}")
        return
    if args.name not in poses:
        parser.error(f"没有位姿 '{args.name}',可选:{list(poses)}")

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    from harness.motion.joint_detour import JointDetour
    from harness.motion.joints import GROUPS, parse_groups
    from harness.motion.whole_body import WholeBody

    pose = poses[args.name]
    try:
        groups = parse_groups(args.groups or ",".join(pose["groups"]))
    except ValueError as e:
        parser.error(str(e))
    wanted = {j for g in groups for j in GROUPS[g][1]}
    target = {j: float(v) for j, v in pose["joints"].items() if j in wanted}

    rclpy.init()
    node = rclpy.create_node("robot_harness_goto_pose")
    body = WholeBody(node)
    urdf = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/robot_description", lambda m: urdf.setdefault("xml", m.data), latched)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                q0 = body.snapshot()
                if "xml" in urdf:
                    break
            except RuntimeError:
                pass
            time.sleep(0.05)
        else:
            sys.exit("5s 没收到 /joint_states 或 /robot_description,hardware.launch.py 在跑吗?")

        for g in groups:
            joints = [j for j in GROUPS[g][1] if j in target]
            if joints:
                delta = max(abs(target[j] - q0[j]) for j in joints)
                print(f"  {g:<10} 最大关节变化 {math.degrees(delta):6.1f}°")
        what = f"全身移到 '{args.name}'({'+'.join(groups)}),速度 {args.speed} rad/s"
        if args.confirm:
            if input(f"{what}。输入 y 执行:").strip().lower() != "y":
                print("已取消,机器人没有动")
                return
        else:
            print(f"[自动执行] {what}")

        detour = JointDetour(body._valid, joint_limits(urdf["xml"]),
                             {n: p["joints"] for n, p in poses.items()}, log=print)
        try:
            ok, msg = body.goto(target, speed=args.speed, progress=print, detour=detour)
        except KeyboardInterrupt:
            body.cancel()
            ok, msg = False, "已取消(Ctrl-C)"
        print(("完成:" if ok else "失败:") + msg)
        sys.exit(0 if ok else 1)
    finally:
        executor.shutdown(timeout_sec=2.0)      # 先停 spin 线程,再销毁节点,否则退出时会崩
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
