"""仿真里控制末端:夹爪开合、吸盘吸放、物体复位。panel.py(按钮)和 ee.py(命令行)共用。

走的都是和真机一样的接口,不走后门:
  夹爪  harness 的 GdkGripper(和大模型的 gripper 工具同一份代码)→ /gdk/joint_control 伺服 → 仿真
  吸盘  suction_control 的 sim 驱动(sim_left / sim_right)→ /sim/suction/<arm> → 仿真
  复位  /sim/reset_objects(仿真才有)
装的是什么末端,从 /robot_description 认(harness/motion/kin.py 的 end_effector),和 harness 一样。
要在仿真的 ROS 环境里用:source ~/robot_harness/sim/sim_env.sh
"""
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness.backends.gripper_gdk import GdkGripper  # noqa: E402
from harness.motion.kin import end_effector  # noqa: E402

ARMS = ("left", "right")
ARM_NAME = {"left": "左手", "right": "右手"}
EE_NAME = {"omnipicker": "夹爪", "suction": "吸盘", "none": "没装"}


class EeControl:
    def __init__(self, node_name="sim_ee_control"):
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()
        self.urdf = None
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/robot_description", self._on_urdf, latched)
        self.reset_client = self.node.create_client(Trigger, "/sim/reset_objects")
        cfg = yaml.safe_load((ROOT / "configs/harness.yaml").read_text(encoding="utf-8"))["motion"]["gripper"]
        self.gripper_cfg = dict(cfg, read_position=False)
        self.gripper = None
        self.suctions = {}

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:
            pass

    def _on_urdf(self, msg):
        self.urdf = msg.data

    def end_effectors(self, timeout_s=10.0):
        """{arm: "omnipicker" / "suction" / "none"};等 /robot_description(仿真刚起来时要等几秒)。"""
        deadline = time.monotonic() + timeout_s
        while self.urdf is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.urdf is None:
            raise RuntimeError(f"{timeout_s}s 没收到 /robot_description(仿真在跑吗?sim_env.sh source 了吗?)")
        return {arm: end_effector(self.urdf, arm) for arm in ARMS}

    def gripper_command(self, arm, action):
        """action: open / close。返回 (ok, 一句话)。"""
        ee = self.end_effectors()
        if ee[arm] != "omnipicker":
            return False, f"{ARM_NAME[arm]}装的是{EE_NAME[ee[arm]]},没有夹爪"
        if self.gripper is None:
            models = {a: ("omnipicker" if ee[a] == "omnipicker" else "") for a in ARMS}
            self.gripper = GdkGripper(self.node, dict(self.gripper_cfg, sim_models=models))
        result = self.gripper.command(arm, action)
        if result.get("ok"):
            return True, f"{ARM_NAME[arm]}夹爪{'张开' if action == 'open' else '闭合'}"
        return False, result.get("error", str(result))

    def _suction(self, arm):
        if arm not in self.suctions:
            from suction_control import make
            self.suctions[arm] = make(f"sim_{arm}")
        return self.suctions[arm]

    def suction_command(self, arm, action):
        """action: on(吸)/ off(放)。返回 (ok, 一句话)。"""
        ee = self.end_effectors()
        if ee[arm] != "suction":
            return False, f"{ARM_NAME[arm]}装的是{EE_NAME[ee[arm]]},没有吸盘"
        s = self._suction(arm)
        try:
            if action == "on":
                s.on()
                got, st = s.wait("attached", 1.0)
                if got:
                    return True, f"{ARM_NAME[arm]}吸盘吸住了 {st.get('attached_object')}"
                return True, f"{ARM_NAME[arm]}吸盘在吸,还没碰到东西(碰到就会吸住)"
            s.off()
            return True, f"{ARM_NAME[arm]}吸盘放开"
        except Exception as e:
            return False, f"{ARM_NAME[arm]}吸盘:{e}"

    def suction_status(self, arm):
        st = self._suction(arm).status()
        if not st.get("connected"):
            return f"连不上:{st.get('error', '')}"
        if st.get("attached"):
            return f"吸住 {st.get('attached_object')}"
        return "在吸(没碰到东西)" if st.get("sucking") else "关"

    def reset_objects(self):
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            return False, "/sim/reset_objects 不可用(仿真在跑吗?)"
        future = self.reset_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, "复位没回应"
        return future.result().success, future.result().message

    def close(self):
        """先停 spin 线程、等它退出,再关节点和 rclpy(顺序反了进程退出时会报 terminate)。"""
        for suction in self.suctions.values():
            if hasattr(suction, "close"):
                suction.close()
        self.executor.shutdown(timeout_sec=1.0)
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()
