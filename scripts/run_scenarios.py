"""场景测试:用会动的假机器人(scenarios/fake_world.py)+ 真的大模型跑 scenarios/*.yaml,统计每条检查的通过率。
机器人不会动,但会调用 DeepSeek(要 .env 里的 key,会花一点钱)。

    python3 scripts/run_scenarios.py                               # 所有场景,新 SYSTEM + 经验,每个跑 3 遍
    python3 scripts/run_scenarios.py handover pick --runs 5        # 只跑名字里带 handover / pick 的
    python3 scripts/run_scenarios.py --variant baseline            # 对照组:旧 SYSTEM(scenarios/baseline_system.txt),没有经验
    python3 scripts/run_scenarios.py --variant no_experience       # 新 SYSTEM,但不给经验

每次运行的对话存到 traces/scenarios/<时间>_<变体>/<场景>_<第几遍>.jsonl。

场景文件格式(scenarios/*.yaml):
    name: handover
    tasks: ["把鼠标放到我手上"]            # 多条 = 连续几条指令(同一个机器人,带前面的对话)
    objects: {mouse: [[0.65, -0.20, 0.72]], hand: [[0.76, -0.05, 0.77]]}
    checks:
      - {label: 不用 touch 碰手, level: must, type: not_called, tool: touch_object, args: {object_id: "hand*"}}
      - {label: ..., type: called / not_called / in_order / after / answer_not_contains, ...}
    检查类型:
      called / not_called  有 / 没有一次调用匹配 tool + args(args 只比给出的键;字符串以 * 结尾 = 前缀匹配)
      in_order             calls 列表里的调用按顺序出现(中间可以夹别的)
      after                最后一次 first 之后有 then(例如松开夹爪之后用 find_objects 确认)
      answer_not_contains  最后的回答里不能有这些词
      任何检查加 negate: true 就反过来
    level:must(默认)/ should;must 全过才算这一遍通过
"""
import argparse
import json
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scenarios"))

from fake_world import FakeWorld  # noqa: E402
from harness.experience import ExperienceBook  # noqa: E402
from harness.gateway import Gateway  # noqa: E402
from harness.llm import DeepSeekLLM  # noqa: E402
from harness.loop import build_system, run_agent  # noqa: E402
from harness.tools.experience import make_experience_tools  # noqa: E402
from harness.tools.objects import make_tools  # noqa: E402
from run_task import load_env, load_poses  # noqa: E402


class Recorder:
    """包一层 Gateway:记下每次调用(名字、参数、结果)。"""

    def __init__(self, gateway):
        self.gateway = gateway
        self.calls = []

    def execute(self, name, args):
        result = self.gateway.execute(name, args)
        self.calls.append({"tool": name, "args": args, "result": result})
        return result


# ---------------- 检查 ----------------
def value_matches(want, got):
    if isinstance(want, str) and want.endswith("*"):
        return isinstance(got, str) and got.startswith(want[:-1])
    return want == got


def call_matches(pattern, call):
    tools = pattern["tool"]
    if isinstance(tools, str):
        tools = [tools]
    if call["tool"] not in tools:                       # tool 可以是一个名字,也可以是几个名字的列表
        return False
    for key, want in (pattern.get("args") or {}).items():
        if not value_matches(want, call["args"].get(key)):
            return False
    return True


def first_index(pattern, calls, start=0):
    for i in range(start, len(calls)):
        if call_matches(pattern, calls[i]):
            return i
    return -1


def check(rule, calls, answer):
    kind = rule["type"]
    if kind == "called":
        return first_index(rule, calls) >= 0
    if kind == "not_called":
        return first_index(rule, calls) < 0
    if kind == "in_order":
        position = 0
        for pattern in rule["calls"]:
            found = first_index(pattern, calls, position)
            if found < 0:
                return False
            position = found + 1
        return True
    if kind == "after":
        last_first = -1
        for i, call in enumerate(calls):
            if call_matches(rule["first"], call):
                last_first = i
        if last_first < 0:
            return False
        return first_index(rule["then"], calls, last_first + 1) >= 0
    if kind == "answer_not_contains":
        for word in rule["words"]:
            if word in answer:
                return False
        return True
    raise ValueError(f"不认识的检查类型 {kind}")


# ---------------- 跑一遍 ----------------
def run_once(scenario, variant, llm, cfg, poses, book, trace_path):
    backend = FakeWorld(scenario["objects"])
    use_experience = variant == "new"
    gateway = Gateway(backend, cfg["motion"], experience=book if use_experience else None, poses=poses)
    recorder = Recorder(gateway)
    notes = {name: pose.get("note", "") for name, pose in poses.items()}
    tools = make_tools(list(poses), notes)
    if use_experience:
        tools = tools + make_experience_tools(book.names())
        system = build_system(book.catalog_text())
    elif variant == "no_experience":
        system = build_system()
    else:
        system = (ROOT / "scenarios" / "baseline_system.txt").read_text(encoding="utf-8")
    history = []
    answer = ""
    with open(trace_path, "w", encoding="utf-8") as trace:
        for task in scenario["tasks"]:
            gateway.moves = 0
            answer = run_agent(task, llm, recorder, tools, max_steps=cfg["agent"]["max_steps"], trace=trace,
                               system=system, history=history)
            history = history + [{"role": "user", "content": task}, {"role": "assistant", "content": answer}]
    results = []
    for rule in scenario["checks"]:
        ok = check(rule, recorder.calls, answer)
        if rule.get("negate"):                          # negate: true = 这条反过来(“不应该出现”)
            ok = not ok
        results.append(ok)
    return results, recorder.calls, answer


def summarize_calls(calls):
    parts = []
    for call in calls:
        args = call["args"]
        short = ""
        if "object_id" in args:
            short = args["object_id"]
        elif "action" in args:
            short = args["action"]
        elif "name" in args:
            short = args["name"]
        elif "query" in args:
            short = args["query"]
        parts.append(f"{call['tool']}({short})")
    return " → ".join(parts)


def main():
    parser = argparse.ArgumentParser(description="场景测试(假机器人 + 真大模型)")
    parser.add_argument("names", nargs="*", help="只跑名字里带这些词的场景")
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--variant", choices=["new", "no_experience", "baseline"], default="new")
    args = parser.parse_args()

    load_env(ROOT / ".env")
    cfg = yaml.safe_load((ROOT / "configs/harness.yaml").read_text(encoding="utf-8"))
    llm = DeepSeekLLM(**cfg["llm"])
    poses = load_poses()
    book = ExperienceBook(ROOT / "experience")
    out = ROOT / "traces" / "scenarios" / f"{time.strftime('%Y%m%d_%H%M%S')}_{args.variant}"
    out.mkdir(parents=True)

    files = sorted((ROOT / "scenarios").glob("*.yaml"))
    total_pass = 0
    total_runs = 0
    for path in files:
        scenario = yaml.safe_load(path.read_text(encoding="utf-8"))
        if args.names and not any(n in scenario["name"] for n in args.names):
            continue
        print(f"\n=== {scenario['name']}:{' / '.join(scenario['tasks'])}", flush=True)
        counts = [0] * len(scenario["checks"])
        passed_runs = 0
        for k in range(args.runs):
            trace_path = out / f"{scenario['name']}_{k + 1}.jsonl"
            try:
                results, calls, answer = run_once(scenario, args.variant, llm, cfg, poses, book, trace_path)
            except RuntimeError as e:
                print(f"  第 {k + 1} 遍出错:{e}", flush=True)
                continue
            must_ok = True
            for i, (rule, ok) in enumerate(zip(scenario["checks"], results)):
                counts[i] += int(ok)
                if not ok and rule.get("level", "must") == "must":
                    must_ok = False
            passed_runs += int(must_ok)
            print(f"  第 {k + 1} 遍 {'通过' if must_ok else '不通过'}:{summarize_calls(calls)}", flush=True)
            print(f"      回答:{answer[:100].replace(chr(10), ' ')}", flush=True)
        for rule, count in zip(scenario["checks"], counts):
            print(f"  [{rule.get('level', 'must'):6}] {count}/{args.runs}  {rule['label']}", flush=True)
        total_pass += passed_runs
        total_runs += args.runs
    print(f"\n变体 {args.variant}:must 全过的 {total_pass}/{total_runs} 遍;对话在 {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
