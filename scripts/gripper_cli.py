"""单独测夹爪(不经过大模型):读状态、打开、闭合。先用它确认夹爪接口能用,再让 harness 用。

  source ~/agi@g2_robot/env_g2.sh
  source ~/.cache/agibot/app/env.sh
  python3 scripts/gripper_cli.py status                         两只手的型号、错误码、位置(只读)
  python3 scripts/gripper_cli.py open  --arm right              打开右手夹爪(直接执行;加 --confirm 按 y 才动)
  python3 scripts/gripper_cli.py close --arm right
  python3 scripts/gripper_cli.py open  --arm right --interface move_ee_pos   换另一种下发方式
"""
import argparse
import sys
import threading
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description="G2 夹爪:status / open / close")
    parser.add_argument("action", choices=["status", "open", "close"])
    parser.add_argument("--arm", choices=["left", "right"], default="right")
    parser.add_argument("--interface", choices=["servo", "move_ee_pos"], help="不填就用 harness.yaml 里的")
    parser.add_argument("--confirm", action="store_true", help="执行前等人按 y(默认直接执行)")
    args = parser.parse_args()

    cfg = yaml.safe_load((ROOT / "configs/harness.yaml").read_text(encoding="utf-8"))["motion"]["gripper"]
    if args.interface:
        cfg["interface"] = args.interface

    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    from harness.backends.gripper_gdk import GdkGripper

    rclpy.init()
    node = rclpy.create_node("robot_harness_gripper_cli")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        gripper = GdkGripper(node, cfg)
        if args.action == "status":
            for arm in ("left", "right"):
                print(arm, gripper.state(arm))
            return
        print("动作前:", gripper.state(args.arm))
        what = f"{args.arm} 夹爪 {args.action}(方式 {cfg['interface']})"
        if args.confirm:
            if input(f"{what}。输入 y 执行:").strip().lower() != "y":
                print("已取消,夹爪没有动")
                return
        else:
            print(f"[自动执行] {what}")
        print("结果:", gripper.command(args.arm, args.action))
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
