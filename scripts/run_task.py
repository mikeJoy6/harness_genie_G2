"""启动 harness。

  python3 scripts/run_task.py "右手去手机上方"                    假机器人
  python3 scripts/run_task.py --real --plan-only "..."             真机:只规划,不运动
  python3 scripts/run_task.py --real                               真机:交互模式,一行一条指令(运动自动执行)
  python3 scripts/run_task.py --real --confirm                     真机:每次运动前按 y
"""
import argparse
import os
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness.backends.fake import FakeBackend
from harness.gateway import Gateway
from harness.llm import DeepSeekLLM
from harness.loop import run_agent
from harness.tools.objects import make_tools


def load_env(path):
    """读取 .env;里面填了值的项优先于终端的环境变量。"""
    if not path.exists():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            key, value = line.split("=", 1)
            value = value.strip().strip("'\"")
            if value:
                os.environ[key.strip()] = value


def ask_user(prompt):
    """--confirm / confirm_motion: true 时:打印计划,等人按 y。"""
    return input(f"{prompt}。输入 y 执行:").strip().lower() == "y"


def announce(prompt):
    """默认:只打印要做什么,直接执行。"""
    print(f"  [自动执行] {prompt}", flush=True)
    return True


def load_poses():
    """configs/poses.yaml:录好的全身位姿(scripts/goto_pose.py 也用它)。没有这个文件就返回空字典。"""
    path = ROOT / "configs/poses.yaml"
    if not path.exists():
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def run_one(task, llm, backend, cfg, args):
    poses = load_poses()                                    # 每条指令重读,录了新位姿不用重启
    # gateway
    confirm = None                                          # 假机器人:不问也不打印
    if args.real:
        if args.confirm or cfg["motion"]["confirm_motion"]:
            confirm = ask_user
        else:
            confirm = announce
    photo = dict(cfg["vision"]["photo"], dir=str(ROOT / cfg["vision"]["photo"]["dir"]))
    gateway = Gateway(backend, cfg["motion"], confirm=confirm, photo_cfg=photo,
                      plan_only=args.plan_only, poses=poses)
    trace_path = ROOT / "traces" / f"{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
    with open(trace_path, "w", encoding="utf-8") as trace:
        notes = {name: pose.get("note", "") for name, pose in poses.items()}
        answer = run_agent(task, llm, gateway, make_tools(list(poses), notes),
                           max_steps=cfg["agent"]["max_steps"], trace=trace)
    print(f"\n机器人助手:{answer}\n[trace] {trace_path}\n", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("task", nargs="?", help="指令;不填则进入交互模式")
    parser.add_argument("--real", action="store_true", help="连接真机")
    parser.add_argument("--plan-only", action="store_true", help="只规划,绝不运动")
    parser.add_argument("--confirm", action="store_true", help="每次运动前等人按 y(默认自动执行)")
    args = parser.parse_args()

    load_env(ROOT / ".env")
    cfg = yaml.safe_load((ROOT / "configs/harness.yaml").read_text(encoding="utf-8"))
    llm = DeepSeekLLM(**cfg["llm"])

    if args.real:
        from harness.backends.ros import RosBackend    # 放这里,假机器人模式才不需要 ROS
        poses = {name: pose["joints"] for name, pose in load_poses().items()}   # 关节绕行时的中转候选
        backend = RosBackend(cfg["vision"], cfg["motion"]["speed_m_s"], debug_dir=ROOT / "traces", poses=poses,
                             gripper_cfg=cfg["motion"]["gripper"], touch_cfg=cfg["motion"]["touch"],
                             tool_axis_tolerance_deg=cfg["motion"]["tool_axis_tolerance_deg"],
                             body_assist_cfg=cfg["motion"]["body_assist"])
    else:
        backend = FakeBackend()

    try:
        if args.task:
            run_one(args.task, llm, backend, cfg, args)
            return
        while True:
            try:
                task = input("你:").strip()
            except EOFError:
                return
            if task in ("q", "quit", "exit"):
                return
            if task:
                run_one(task, llm, backend, cfg, args)
    finally:
        backend.close()


if __name__ == "__main__":
    main()
