"""把机器人当前的关节角录进 configs/poses.yaml。只读 /joint_states,机器人不会动。

  source ~/agi@g2_robot/env_g2.sh
  python3 scripts/record_pose.py right_now --groups right_arm                 录当前右臂,存成位姿 right_now
  python3 scripts/record_pose.py right_now --groups right_arm --mirror-left   同时生成左臂对称位姿 right_now_mirror
  python3 scripts/goto_pose.py right_now_mirror                               左臂去对称位姿(会问 y)
  python3 scripts/record_pose.py init --note 初始位姿 --force                 全身当前姿态录成 init;
                                                                              对 harness 说“抵达初始位姿”就 go_to_pose(init)

左右对称(相对躯干的左右镜像)的关节换算,用 G2 URDF 正运动学验证过(误差 1e-5 m):
  左臂关节 i = MIRROR_SIGN[i] × 右臂关节 i,MIRROR_SIGN = (-1, +1, -1, +1, -1, +1, -1)
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
MIRROR_SIGN = (-1, 1, -1, 1, -1, 1, -1)
RIGHT = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]
LEFT = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]


def mirror_right_to_left(joints):
    """右臂关节角 → 左臂的对称关节角。"""
    left = {}
    for sign, r, l in zip(MIRROR_SIGN, RIGHT, LEFT):
        left[l] = round(sign * joints[r], 5)
    return left


def check_limits(target, limits):
    """超出 URDF 限位的关节,返回说明列表。"""
    bad = []
    for name, value in target.items():
        if name in limits:
            low, high = limits[name]
            if not low <= value <= high:
                bad.append(f"{name} = {math.degrees(value):.1f}°,限位 [{math.degrees(low):.1f}°, {math.degrees(high):.1f}°]")
    return bad


def main():
    parser = argparse.ArgumentParser(description="把当前关节角录进 configs/poses.yaml(机器人不动)")
    parser.add_argument("name", help="位姿名")
    parser.add_argument("--groups", default="body,head,left_arm,right_arm", help="录哪些组,逗号分隔")
    parser.add_argument("--mirror-left", action="store_true",
                        help="再生成一个 <名字>_mirror:左臂摆成右臂的左右对称")
    parser.add_argument("--force", action="store_true", help="同名位姿已存在时覆盖")
    parser.add_argument("--note", help="位姿的含义,会写进 go_to_pose 的工具说明给大模型看,例如 \"初始位姿\"")
    args = parser.parse_args()

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    from harness.motion import kin  # noqa: F401  导入时屏蔽 URDF 解析器对 G2 扩展标签的告警
    from harness.motion.joints import GROUPS, parse_groups
    from harness.motion.whole_body import WholeBody
    from urdf_parser_py.urdf import URDF

    try:
        groups = parse_groups(args.groups)
    except ValueError as e:
        parser.error(str(e))
    if args.mirror_left and "right_arm" not in groups:
        parser.error("--mirror-left 要录右臂:--groups 里要有 right_arm")

    poses = yaml.safe_load(POSES_PATH.read_text(encoding="utf-8")) or {}
    new_names = [args.name] + ([f"{args.name}_mirror"] if args.mirror_left else [])
    exists = [n for n in new_names if n in poses]
    if exists and not args.force:
        sys.exit(f"位姿 {exists} 已存在,换个名字或加 --force 覆盖")

    rclpy.init()
    node = rclpy.create_node("robot_harness_record_pose")
    body = WholeBody(node)
    urdf = {}

    def on_urdf(msg):
        urdf["xml"] = msg.data

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/robot_description", on_urdf, latched)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        q = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                q = body.snapshot()
                if "xml" in urdf:
                    break
            except RuntimeError:
                pass
            time.sleep(0.05)
        if q is None or "xml" not in urdf:
            sys.exit("5s 没收到 /joint_states 或 /robot_description,hardware.launch.py 在跑吗?")
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()

    recorded = {j: round(float(q[j]), 5) for g in groups for j in GROUPS[g][1]}
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    note = args.note if args.note else f"record_pose.py 录于 {stamp}"
    poses[args.name] = {"groups": groups, "joints": recorded, "note": note}
    print(f"录下 {args.name}({'+'.join(groups)}):")
    for g in groups:
        print(f"  {g:<10} " + "  ".join(f"{math.degrees(recorded[j]):7.1f}" for j in GROUPS[g][1]) + "  (度)")

    if args.mirror_left:
        left = mirror_right_to_left(recorded)
        limits = {j.name: (j.limit.lower, j.limit.upper) for j in URDF.from_xml_string(urdf["xml"]).joints
                  if j.limit is not None and j.type != "continuous"}
        bad = check_limits(left, limits)
        if bad:
            sys.exit("右臂的对称姿态超出左臂限位,没有保存:\n  " + "\n  ".join(bad))
        poses[f"{args.name}_mirror"] = {"groups": ["left_arm"], "joints": left,
                                        "note": f"{args.name} 右臂的左右对称,{stamp}"}
        print(f"生成 {args.name}_mirror(left_arm):")
        print("  left_arm   " + "  ".join(f"{math.degrees(left[j]):7.1f}" for j in LEFT) + "  (度)")

    POSES_PATH.write_text(yaml.safe_dump(poses, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"已写入 {POSES_PATH}")


if __name__ == "__main__":
    main()
