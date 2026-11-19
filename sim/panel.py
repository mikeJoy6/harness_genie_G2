"""仿真末端控制面板:每只手一列按钮(按装的末端生成)+ 物体复位。sim.launch.py 默认会开(panel:=false 不开)。

单独开:source ~/robot_harness/sim/sim_env.sh && python3 ~/robot_harness/sim/panel.py
走的接口和真机一样,见 sim/ee_control.py。
"""
import queue
import signal
import sys
import threading
import tkinter as tk
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from ee_control import ARM_NAME, ARMS, EE_NAME, EeControl  # noqa: E402

BUTTONS = {
    "omnipicker": [("张开", "gripper", "open"), ("闭合", "gripper", "close")],
    "suction": [("吸", "suction", "on"), ("放", "suction", "off")],
    "none": [],
}


class Panel:
    def __init__(self):
        self.ctl = EeControl("sim_ee_panel")
        self.results = queue.Queue()          # 后台线程的结果 → 主线程显示(tkinter 只能在主线程改界面)
        self.root = tk.Tk()
        self.root.title("G2 仿真末端")
        self.root.attributes("-topmost", True)
        self.status = tk.StringVar(value="等 /robot_description ……")
        self.suction_labels = {}
        try:
            ee = self.ctl.end_effectors()
        except RuntimeError as e:
            ee = {arm: "none" for arm in ARMS}
            self.status.set(str(e))
        else:
            self.status.set("就绪")
        for col, arm in enumerate(ARMS):
            frame = tk.LabelFrame(self.root, text=f"{ARM_NAME[arm]}:{EE_NAME[ee[arm]]}", padx=8, pady=6)
            frame.grid(row=0, column=col, padx=6, pady=6, sticky="n")
            for text, kind, action in BUTTONS[ee[arm]]:
                tk.Button(frame, text=text, width=8, command=self.button(kind, arm, action)).pack(pady=2)
            if ee[arm] == "suction":
                self.suction_labels[arm] = tk.StringVar(value="……")
                tk.Label(frame, textvariable=self.suction_labels[arm], fg="gray30").pack(pady=2)
            if ee[arm] == "none":
                tk.Label(frame, text="(没有可控制的末端)", fg="gray40").pack(pady=2)
        tk.Button(self.root, text="物体复位", command=self.button("reset", None, None)).grid(
            row=1, column=0, columnspan=2, pady=4)
        tk.Label(self.root, textvariable=self.status, fg="blue", wraplength=320).grid(
            row=2, column=0, columnspan=2, padx=6, pady=6)
        self.root.after(200, self.poll)
        self.root.after(500, self.refresh_suction)
        # rclpy 会接管 Ctrl+C / SIGTERM(只关 ROS,不退进程),窗口就关不掉(ros2 launch 退出时也是)。
        #   这里换成:收到信号就关窗口。信号处理在 Python 拿到控制权时执行,poll 每 200ms 回一次 Python
        signal.signal(signal.SIGINT, self.on_signal)
        signal.signal(signal.SIGTERM, self.on_signal)

    def on_signal(self, signum, frame):
        self.root.after(0, self.root.destroy)

    def button(self, kind, arm, action):
        """给按钮用的回调:按下时执行 run(kind, arm, action)。"""
        def on_press():
            self.run(kind, arm, action)
        return on_press

    def run(self, kind, arm, action):
        """按钮回调:动作放到后台线程做(夹爪一次要 ~2s),窗口不卡。"""
        self.status.set("执行中……")
        threading.Thread(target=self.work, args=(kind, arm, action), daemon=True).start()

    def work(self, kind, arm, action):
        try:
            if kind == "gripper":
                ok, text = self.ctl.gripper_command(arm, action)
            elif kind == "suction":
                ok, text = self.ctl.suction_command(arm, action)
            else:
                ok, text = self.ctl.reset_objects()
        except Exception as e:
            ok, text = False, str(e)
        self.results.put(("status", None, ("✓ " if ok else "✗ ") + text))

    def poll(self):
        while not self.results.empty():
            kind, arm, text = self.results.get()
            if kind == "status":
                self.status.set(text)
            else:
                self.suction_labels[arm].set(text)
        self.root.after(200, self.poll)

    def refresh_suction(self):
        if self.suction_labels:
            threading.Thread(target=self.read_suction, daemon=True).start()
        self.root.after(1000, self.refresh_suction)

    def read_suction(self):
        for arm in self.suction_labels:
            try:
                text = self.ctl.suction_status(arm)
            except Exception as e:
                text = f"读不到:{e}"
            self.results.put(("suction", arm, text))

    def mainloop(self):
        try:
            self.root.mainloop()
        finally:
            self.ctl.close()


if __name__ == "__main__":
    Panel().mainloop()
