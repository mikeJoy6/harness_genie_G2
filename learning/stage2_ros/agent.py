"""大模型 + ROS:用中文问机器人的状态,大模型调用工具去读真实数据。

    python3 learning/stage2_ros/agent.py                        交互模式,q 退出
    python3 learning/stage2_ros/agent.py "左手末端现在在哪?"     只问一句

循环和第一阶段的 agent_loop.py 一样,只是工具换成了读 ROS 的版本。只读,机器人不会动。
"""
import json
import sys

import llm
from ros_io import RosIO
from tools import TOOLS, make_handlers

SYSTEM = ("你是智元 G2 机器人的助手,只能通过工具了解机器人状态,不能让机器人运动。"
          "没有合适的工具时,直接说明做不到。回答时给出数字和单位。")


def run_tool(handlers, tool_name, raw_args):
    func = handlers.get(tool_name)
    if func is None:
        return {"ok": False, "error": f"没有这个工具:{tool_name}"}
    try:
        return func(**json.loads(raw_args))
    except Exception as e:                    # TF 查不到、关节名不对等,都交给模型
        return {"ok": False, "error": f"{type(e).__name__}: {e}"}


def run_agent(key, handlers, task, max_steps=6):
    messages = [{"role": "system", "content": SYSTEM}, {"role": "user", "content": task}]
    for step in range(max_steps):
        reply = llm.chat(key, messages, TOOLS)
        messages.append(reply)
        if not reply.get("tool_calls"):
            return reply["content"]
        for call in reply["tool_calls"]:
            name, raw_args = call["function"]["name"], call["function"]["arguments"]
            result = run_tool(handlers, name, raw_args)
            print(f"  [第{step}步] {name}({raw_args}) -> {result}")
            messages.append({"role": "tool", "tool_call_id": call["id"],
                             "content": json.dumps(result, ensure_ascii=False)})
    return f"已经用完 {max_steps} 步,任务没有完成"


def main():
    key = llm.read_key()
    with RosIO("learning_agent") as ros:
        ros.wait_ready()
        handlers = make_handlers(ros)

        if len(sys.argv) > 1:                 # 命令行带了问题:只问这一句
            print("大模型:", run_agent(key, handlers, " ".join(sys.argv[1:])))
            return

        print("ROS 已连接。输入问题,q 退出。")
        while True:
            try:
                task = input("你: ").strip()
            except EOFError:
                return
            if task.lower() in ("q", "quit", "exit"):
                return
            if task:
                print("大模型:", run_agent(key, handlers, task))


if __name__ == "__main__":
    main()
