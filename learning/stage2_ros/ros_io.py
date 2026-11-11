"""和 ROS 打交道的部分全在这一个类里。其他文件只调用它的方法,不碰 rclpy。

RosIO 做四件事:
    1. 订阅 /joint_states        → joints()
    2. 订阅 /robot_description   → urdf_xml()
    3. 监听 TF                    → lookup(parent, child)
    4. 调用 MoveIt 碰撞检查服务  → check_collision(changes)
全部只读,不发布任何控制指令,机器人不会动。
"""
import threading
import time

import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

import config


class RosIO:
    def __init__(self, node_name="learning_ros_io"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)

        # 回调在后台线程里写,主线程读 → 用锁保护,避免读到写了一半的数据
        self._lock = threading.Lock()
        self._joints = {}            # 关节名 → 弧度
        self._joints_time = None     # 最近一次收到 /joint_states 的时间
        self._urdf = None

        # 1. 关节角。GDK 用 BEST_EFFORT 发布,订阅必须用 sensor_data QoS,否则一条都收不到
        self.node.create_subscription(JointState, config.JOINT_STATES_TOPIC, self._on_joints,
                                      qos_profile_sensor_data)

        # 2. URDF。TRANSIENT_LOCAL = 晚启动的订阅者也能收到之前发过的那条
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, config.URDF_TOPIC, self._on_urdf, latched)

        # 3. TF。Buffer 存最近一段时间所有坐标变换,Listener 负责往里填
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # 4. MoveIt 碰撞检查服务的客户端
        self.validity_client = self.node.create_client(GetStateValidity, config.VALIDITY_SERVICE)

        # 在后台线程里一直 spin:回调会自动被调用,主线程可以随时读数据
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ---------------- 回调(后台线程里被 ROS 调用) ----------------
    def _on_joints(self, msg):
        with self._lock:
            self._joints.update(zip(msg.name, msg.position))
            self._joints_time = time.monotonic()

    def _on_urdf(self, msg):
        with self._lock:
            self._urdf = msg.data

    # ---------------- 给其他文件用的方法 ----------------
    def wait_ready(self, timeout_s=config.READY_TIMEOUT_S):
        """等到关节角和 URDF 都收到。超时就报错,并说清楚缺哪个。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._joints and self._urdf:
                    return
            time.sleep(0.05)
        missing = [name for name, got in [(config.JOINT_STATES_TOPIC, self._joints),
                                          (config.URDF_TOPIC, self._urdf)] if not got]
        raise RuntimeError(f"{timeout_s}s 没收到 {', '.join(missing)}。机器人(或 fake_robot.py)在跑吗?"
                           f"ROS_DOMAIN_ID 对吗?")

    def joints(self, names=None):
        """返回 {关节名: 弧度}。names 不填就返回全部。"""
        with self._lock:
            if names is None:
                return dict(self._joints)
            missing = [n for n in names if n not in self._joints]
            if missing:
                raise KeyError(f"/joint_states 里没有这些关节:{missing}")
            return {n: self._joints[n] for n in names}

    def joints_age_s(self):
        """距离上一次收到 /joint_states 过了多少秒。太大说明数据断了。"""
        with self._lock:
            return None if self._joints_time is None else time.monotonic() - self._joints_time

    def urdf_xml(self):
        with self._lock:
            return self._urdf

    def lookup(self, parent, child, timeout_s=config.TF_TIMEOUT_S):
        """查 child 在 parent 坐标系下的位姿。返回 (xyz 米, 四元数 [x, y, z, w])。"""
        tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(),   # Time() = 最新的一份
                                             timeout=Duration(seconds=timeout_s))
        t, q = tf.transform.translation, tf.transform.rotation
        return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]

    def check_collision(self, changes=None, timeout_s=config.SERVICE_TIMEOUT_S):
        """问 MoveIt:当前姿态(再把 changes 里的关节改成新值)整台机器人会不会自碰撞。

        changes: {关节名: 弧度},只是假设,机器人不会动。
        返回 {"available": bool, "valid": bool, "contacts": [(link1, link2), ...]}
        """
        if not self.validity_client.service_is_ready():
            return {"available": False, "valid": None, "contacts": [],
                    "error": f"{config.VALIDITY_SERVICE} 不可用(move_group 没起?)"}

        state = self.joints()
        state.update(changes or {})
        req = GetStateValidity.Request()
        req.robot_state.joint_state.name = list(state)
        req.robot_state.joint_state.position = list(state.values())
        req.robot_state.is_diff = True      # 在 MoveIt 当前状态的基础上只改关节,其他(附着物等)保留
        req.group_name = ""                 # 空 = 检查整台机器人,不只是一条手臂

        future = self.validity_client.call_async(req)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)               # 后台线程在 spin,回复到了 future 会自动 done
        if not future.done():
            return {"available": True, "valid": None, "contacts": [], "error": "MoveIt 超时没回答"}

        res = future.result()
        contacts = sorted({(c.contact_body_1, c.contact_body_2) for c in res.contacts})
        return {"available": True, "valid": bool(res.valid), "contacts": contacts}

    def close(self):
        """顺序很重要:先停 spin 线程,再销毁节点。反过来会在退出时崩溃。"""
        self._executor.shutdown()
        self._spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # 支持 with RosIO() as ros: ... 离开 with 时自动 close()
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
