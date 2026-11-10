"""启动 harness。

  python3 scripts/run_task.py "让左臂末端靠近红色积木"              假机器人
  python3 scripts/run_task.py --real --plan-only "..."             真机:只规划,不运动
  python3 scripts/run_task.py --real                               真机:交互模式,一行一条指令
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
from harness.colors import load_colors
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
    return input(prompt).strip().lower() == "y"


def run_one(task, llm, backend, cfg, args):
    colors = load_colors(ROOT / "configs/colors.yaml")      # 每条指令重读,取色后不用重启
    # 
    gateway = Gateway(backend, cfg["motion"], colors, confirm=ask_user if args.real else None,
                      plan_only=args.plan_only)
    trace_path = ROOT / "traces" / f"{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
    with open(trace_path, "w", encoding="utf-8") as trace:
        answer = run_agent(task, llm, gateway, make_tools(colors), max_steps=cfg["agent"]["max_steps"], trace=trace)
    print(f"\n机器人助手:{answer}\n[trace] {trace_path}\n", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("task", nargs="?", help="指令;不填则进入交互模式")
    parser.add_argument("--real", action="store_true", help="连接真机")
    parser.add_argument("--plan-only", action="store_true", help="只规划,绝不运动")
    args = parser.parse_args()

    load_env(ROOT / ".env")
    cfg = yaml.safe_load((ROOT / "configs/harness.yaml").read_text(encoding="utf-8"))
    llm = DeepSeekLLM(**cfg["llm"])

    if args.real:
        from harness.backends.ros import RosBackend    # 放这里,假机器人模式才不需要 ROS
        backend = RosBackend(cfg["vision"], cfg["motion"]["speed_m_s"], debug_dir=ROOT / "traces")
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
