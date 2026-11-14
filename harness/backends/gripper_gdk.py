"""G2 夹爪(末端执行器):开 / 合 / 读状态。夹爪不在 ROS 的 /joint_states 里,要通过 GDK 控制。

两种下发方式(configs/harness.yaml 的 motion.gripper.interface 选):
  servo        往 /gdk/joint_control 发 gdk_msgs/JointPositionServo,只带夹爪那一个关节。
               手臂的 ros2_control 插件也在用这个伺服话题;GDK 文档说“和伺服接口同时用时,末端要走伺服接口”,所以默认用它。
  move_ee_pos  GDK Python 接口 Robot.move_ee_pos(非阻塞)。文档注明不能和伺服接口同时用,只在手臂不受控时用。

状态来自 GDK Robot.get_whole_body_status():{left,right}_end_model(型号,空 = 没装 / 没识别)、{left,right}_end_error(错误码)。
夹爪位置来自 Robot.get_end_state();2026-11-12 实测一直报 "No end state message found",所以执行后多半**确认不了到没到位**,
结果里 verified=False 会如实写出来。

运行前要 source ~/.cache/agibot/app/env.sh(agibot_gdk 要它设置库路径)。
"""
import os
import threading
import time
from pathlib import Path

ARMS = ("left", "right")
ARM_NAME = {"left": "左手", "right": "右手"}

# 每种夹爪的 (打开, 闭合) 关节位置,弧度或米,来自 GDK 文档 move_ee_pos 一节
POSITIONS = {
    "omnipicker": (-0.785, 0.0),
    "ctek90d": (-0.91, 0.0),
    "dahuan": (0.0, 0.025),
}
# 伺服接口里的夹爪关节名(~/.cache/agibot/app/gdk/config/joint_mapping.json)
SERVO_JOINT = {"left": "idx31_gripper_l_inner_joint1", "right": "idx71_gripper_r_inner_joint1"}


class GdkGripper:
    def __init__(self, node, cfg):
        """node:已有的 rclpy 节点(servo 方式用它建发布者)。cfg:harness.yaml 的 motion.gripper。"""
        self.node = node
        self.cfg = cfg
        self.lock = threading.Lock()
        self._gdk = None            # agibot_gdk 模块,第一次用时才初始化(gdk_init 要一两秒)
        self._robot = None
        self._pub = None
        # get_end_state:这台 G2 的 HAL 根本不发夹爪状态(/gdk/joint_state 22 个关节、/joint_states 30 个,都没有夹爪),
        # 读一次就阻塞约 2s 并往终端打一行 "E ... No end state message found"。所以默认不读(read_position: false)。
        self._has_end_state = bool(cfg.get("read_position", False))

    # ---------------- 连接 GDK ----------------
    def _connect(self):
        """返回 None 表示连上了,否则返回错误文字。"""
        if self._robot is not None:
            return None
        # GDK 用 glog,默认把 python3.10.INFO、python3.10.log.INFO.<时间>… 写在“当前目录”(以前堆满项目根目录)。
        # 必须在 import agibot_gdk 之前设;用户自己设过 GLOG_log_dir 就不覆盖。
        log_dir = Path(__file__).resolve().parents[2] / "logs" / "gdk"
        log_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("GLOG_log_dir", str(log_dir))
        try:
            import agibot_gdk
        except ImportError:
            return "导入不了 agibot_gdk:启动 harness 前要 source ~/.cache/agibot/app/env.sh"
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            return "GDK 初始化失败(gdk_init),机器人连上了吗?"
        self._gdk = agibot_gdk
        self._robot = agibot_gdk.Robot()
        time.sleep(1.0)             # 等第一包状态
        return None

    def _publisher(self):
        if self._pub is None:
            from gdk_msgs.msg import JointPositionServo
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)      # 和手臂插件发这个话题用的一样
            self._pub = self.node.create_publisher(JointPositionServo, "/gdk/joint_control", qos)
        return self._pub

    # ---------------- 读 ----------------
    def state(self, arm):
        """{"ok", "arm", "model", "error_code", "position"(读不到为 None)}"""
        with self.lock:
            error = self._connect()
            if error:
                return {"ok": False, "arm": arm, "error": error}
            try:
                status = self._robot.get_whole_body_status()
            except Exception as e:
                return {"ok": False, "arm": arm, "error": f"读不到 GDK 整机状态:{e}"}
            return {"ok": True, "arm": arm, "model": status.get(f"{arm}_end_model", ""),
                    "error_code": int(status.get(f"{arm}_end_error", 0)), "position": self._position(arm)}

    def _position(self, arm):
        if not self._has_end_state:
            return None
        try:
            end = self._robot.get_end_state()[f"{arm}_end_state"]
            return float(end["end_states"][0]["position"])
        except Exception:           # 实测常年 "No end state message found"
            self._has_end_state = False
            return None

    # ---------------- 开 / 合 ----------------
    def command(self, arm, action):
        """action:"open" 或 "close"。返回 {"ok", "msg", "verified", ...}。"""
        before = self.state(arm)
        if not before["ok"]:
            return before
        model = before["model"]
        if not model:
            return {"ok": False, "arm": arm,
                    "error": f"{ARM_NAME[arm]}没有识别到夹爪(GDK {arm}_end_model 为空,错误码 {before['error_code']}),"
                             "可能没装或没上电"}
        if model not in POSITIONS:
            return {"ok": False, "arm": arm, "error": f"{ARM_NAME[arm]}的末端是 {model},还不支持(支持 {list(POSITIONS)})"}
        if before["error_code"]:
            return {"ok": False, "arm": arm, "error": f"{ARM_NAME[arm]}夹爪有错误码 {before['error_code']},先排查再用"}

        opened, closed = POSITIONS[model]
        target = opened if action == "open" else closed
        with self.lock:
            if self.cfg["interface"] == "servo":
                self._send_servo(arm, target)
            else:
                error = self._send_move_ee_pos(arm, target)
                if error:
                    return {"ok": False, "arm": arm, "error": error}
        time.sleep(self.cfg["settle_s"])

        after = self.state(arm)
        result = {"arm": arm, "model": model, "action": action, "target_position": target,
                  "interface": self.cfg["interface"]}
        if not after["ok"]:
            return {**result, "ok": False, "error": f"命令发了,但之后读不到状态:{after['error']}"}
        if after["error_code"]:
            return {**result, "ok": False, "error": f"命令发了,夹爪报错误码 {after['error_code']}"}
        if after["position"] is None:
            return {**result, "ok": True, "verified": False,
                    "msg": "命令已发送,夹爪没报错(这台机器人不回报夹爪开合位置,所以没有到位确认,按成功处理)"}
        result["verified"] = True
        result["position"] = round(after["position"], 4)
        if abs(after["position"] - target) <= self.cfg["position_tol"]:
            return {**result, "ok": True, "msg": "已到位"}
        return {**result, "ok": False,
                "error": f"没到位:目标 {target},实际 {after['position']:.3f}(可能夹住了东西,或被挡住)"}

    def _send_servo(self, arm, target):
        """按 servo_rate_hz 连续发 servo_hold_s 秒,只带夹爪一个关节。"""
        from gdk_msgs.msg import JointPositionServo
        pub = self._publisher()
        period = 1.0 / self.cfg["servo_rate_hz"]
        end = time.monotonic() + self.cfg["servo_hold_s"]
        while time.monotonic() < end:
            msg = JointPositionServo()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.control_period = period
            msg.joint_names = [SERVO_JOINT[arm]]
            msg.joint_positions = [float(target)]
            msg.joint_velocities = [0.0]
            pub.publish(msg)
            time.sleep(period)

    def _send_move_ee_pos(self, arm, target):
        g = self._gdk
        states = g.JointStates()
        states.group = f"{arm}_tool"
        joint = g.JointState()
        joint.position = float(target)
        states.states = [joint]
        states.nums = 1
        try:
            result = self._robot.move_ee_pos(states)
        except Exception as e:
            return f"move_ee_pos 失败:{e}"
        if result is not None and result != g.GDKRes.kSuccess:
            return f"move_ee_pos 返回 {result}"
        return None
