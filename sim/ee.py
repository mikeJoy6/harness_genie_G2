"""仿真末端命令行(写脚本用;点按钮用 panel.py)。

    source ~/robot_harness/sim/sim_env.sh
    python3 ~/robot_harness/sim/ee.py                       # 两只手装的什么、吸盘状态
    python3 ~/robot_harness/sim/ee.py right open            # 夹爪:open / close
    python3 ~/robot_harness/sim/ee.py left suck             # 吸盘:suck / release
    python3 ~/robot_harness/sim/ee.py reset                 # 物体放回初始位置

走的接口和真机一样,见 sim/ee_control.py。
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from ee_control import ARM_NAME, ARMS, EE_NAME, EeControl  # noqa: E402

ACTIONS = {"open": ("gripper", "open"), "close": ("gripper", "close"),
           "suck": ("suction", "on"), "release": ("suction", "off")}


def main():
    parser = argparse.ArgumentParser(description="仿真末端:夹爪开合 / 吸盘吸放 / 物体复位")
    parser.add_argument("target", nargs="?", choices=list(ARMS) + ["reset"], help="left / right / reset;不填只看状态")
    parser.add_argument("action", nargs="?", choices=list(ACTIONS))
    args = parser.parse_args()
    ctl = EeControl("sim_ee_cli")
    try:
        if args.target == "reset":
            ok, text = ctl.reset_objects()
        elif args.target is None:
            ee = ctl.end_effectors()
            for arm in ARMS:
                extra = f",{ctl.suction_status(arm)}" if ee[arm] == "suction" else ""
                print(f"{ARM_NAME[arm]}:{EE_NAME[ee[arm]]}({ee[arm]}){extra}")
            return 0
        else:
            if args.action is None:
                parser.error("要给动作:open / close / suck / release")
            kind, action = ACTIONS[args.action]
            if kind == "gripper":
                ok, text = ctl.gripper_command(args.target, action)
            else:
                ok, text = ctl.suction_command(args.target, action)
        print(("✓ " if ok else "✗ ") + text)
        return 0 if ok else 1
    finally:
        ctl.close()


if __name__ == "__main__":
    sys.exit(main())
