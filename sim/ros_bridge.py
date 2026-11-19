"""G2 仿真的 ROS 桥:系统 Python 3.10 + rclpy,冒充 GDK 收发和真机一样的 /gdk/* 话题。
物理在 sim/physics.py(MuJoCo 4,Python 3.12),两边用 Unix 套接字通信(sim/link.py)。

不要直接运行,用 sim/launch/sim.launch.py 启动,见 sim/README.md。

话题(和真机一致的):
  收  /gdk/joint_control           gdk_msgs/JointPositionServo   关节 + 夹爪的目标位置(手臂插件 100Hz 发;夹爪 harness / 面板发)
  发  /gdk/joint_state             gdk_msgs/JointState           22 个关节(不含夹爪,和真机一样),100Hz
  发  /gdk/motion_control_status   gdk_msgs/MotionControlStatus  mode=SERVO;两个末端的位姿和六维力,100Hz
  发  /gdk/tf                      tf2_msgs/TFMessage            base_link → head_link3 / arm_*_end_link,100Hz
  发  /gdk/camera/head_color       sensor_msgs/CompressedImage   640x400 JPEG,15Hz
  发  /gdk/camera/head_depth       sensor_msgs/Image             16UC1 毫米,和彩色逐像素对齐、时间戳相同,15Hz
  发  /gdk/camera/head_stereo_left sensor_msgs/CompressedImage   1920x1536 JPEG(拍照用),1Hz
仿真才有的:
  发  /sim/camera/head_seg         sensor_msgs/Image             16UC1,每个像素是哪个物体(0 = 不是物体),时间戳同上
  发  /sim/objects                 std_msgs/String(latched)      JSON:[{"index", "name", "labels"}],分割图的编号表
  服务 /sim/reset_objects          std_srvs/Trigger              物体放回初始位置(机器人不动)
  服务 /sim/suction/<left|right>         std_srvs/SetBool      开 / 关吸盘(只有装吸盘的手)
  服务 /sim/suction/<left|right>/status  std_srvs/Trigger      吸盘状态 JSON(suction_control 的 sim 驱动用)
"""
import array
import json
import sys
import threading
import time
from pathlib import Path

import rclpy
from gdk_msgs.msg import JointPositionServo, JointState, MotionControlStatus
from geometry_msgs.msg import Pose, TransformStamped, Twist, Wrench
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).parent))
import link  # noqa: E402

END_FRAMES = {"left": "arm_l_end_link", "right": "arm_r_end_link"}


def best_effort_qos():
    """GDK 的话题全是 BEST_EFFORT + VOLATILE(真机插件里写着)。"""
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def latched_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def byte_array(data):
    """ROS 消息的 uint8[] 字段要这样给:直接给 bytes 时 rclpy 逐个元素检查,一张 640x400 深度图要 24ms。"""
    return array.array("B", data)


class Bridge:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("g2_sim_bridge")
        qos = best_effort_qos()
        self.pub_joint = self.node.create_publisher(JointState, "/gdk/joint_state", qos)
        self.pub_status = self.node.create_publisher(MotionControlStatus, "/gdk/motion_control_status", qos)
        self.pub_tf = self.node.create_publisher(TFMessage, "/gdk/tf", qos)
        self.pub_color = self.node.create_publisher(CompressedImage, "/gdk/camera/head_color", qos)
        self.pub_depth = self.node.create_publisher(Image, "/gdk/camera/head_depth", qos)
        self.pub_photo = self.node.create_publisher(CompressedImage, "/gdk/camera/head_stereo_left", qos)
        self.pub_seg = self.node.create_publisher(Image, "/sim/camera/head_seg", qos)
        self.pub_objects = self.node.create_publisher(String, "/sim/objects", latched_qos())
        self.node.create_subscription(JointPositionServo, "/gdk/joint_control", self.on_servo, qos)
        self.node.create_service(Trigger, "/sim/reset_objects", self.on_reset_objects)
        self.suction_status = {}             # arm → {"on", "attached"}(物理进程 10Hz 报)
        self.suction_services = []
        self.conn = None
        self.send_lock = threading.Lock()

    # ---------------- 和物理进程的连接 ----------------
    def connect(self):
        """一直重试到连上(物理进程可能还在建模型 / 开窗口)。"""
        first = True
        while rclpy.ok():
            try:
                conn = link.connect()
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if first:
                    self.node.get_logger().info(f"等物理进程(sim/physics.py)开 {link.SOCKET_PATH} ...")
                    first = False
                time.sleep(0.5)
        else:
            return False
        header, _ = link.recv(conn)
        self.on_hello(header)
        self.conn = conn
        return True

    def send(self, header):
        conn = self.conn
        if conn is None:
            return False
        with self.send_lock:
            try:
                link.send(conn, header)
                return True
            except OSError:
                self.conn = None
                return False

    def run(self):
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
        while rclpy.ok():
            if not self.connect():
                return
            self.node.get_logger().info("已连上物理进程")
            try:
                while True:
                    header, blobs = link.recv(self.conn)
                    self.dispatch(header, blobs)
            except (EOFError, OSError):
                self.node.get_logger().warn("物理进程断开了,等它重启")
                self.conn = None

    def dispatch(self, header, blobs):
        kind = header.get("type")
        if kind == "state":
            self.publish_state(header)
        elif kind == "camera":
            self.publish_camera(header, blobs)
        elif kind == "photo":
            msg = CompressedImage()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = byte_array(blobs[0])
            self.pub_photo.publish(msg)
        elif kind == "suction":
            self.suction_status = header["arms"]

    def on_hello(self, hello):
        table = hello["objects"]
        self.pub_objects.publish(String(data=json.dumps(table, ensure_ascii=False)))
        ee = hello["end_effectors"]
        self.node.get_logger().info(f"末端 {ee},{len(table)} 个物体")
        if not self.suction_services:            # 第一次连上时按末端建吸盘服务
            for arm, kind in ee.items():
                if kind != "suction":
                    continue
                self.suction_services.append(self.node.create_service(
                    SetBool, f"/sim/suction/{arm}", self.make_suction_set(arm)))
                self.suction_services.append(self.node.create_service(
                    Trigger, f"/sim/suction/{arm}/status", self.make_suction_status(arm)))

    # ---------------- ROS → 物理 ----------------
    def on_servo(self, msg):
        targets = {}
        for name, value in zip(msg.joint_names, msg.joint_positions):
            targets[name] = float(value)
        self.send({"type": "targets", "targets": targets})

    def on_reset_objects(self, request, response):
        response.success = self.send({"type": "reset_objects"})
        response.message = "物体已放回初始位置" if response.success else "物理进程没连上"
        return response

    def make_suction_set(self, arm):
        def handler(request, response):
            response.success = self.send({"type": "suction", "arm": arm, "on": bool(request.data)})
            response.message = ("开吸" if request.data else "停吸") if response.success else "物理进程没连上"
            return response
        return handler

    def make_suction_status(self, arm):
        def handler(request, response):
            st = self.suction_status.get(arm)
            response.success = st is not None and self.conn is not None
            response.message = json.dumps(st or {}, ensure_ascii=False)
            return response
        return handler

    # ---------------- 物理 → ROS ----------------
    def publish_state(self, s):
        stamp = self.node.get_clock().now().to_msg()
        n = len(s["names"])
        js = JointState()
        js.header.stamp = stamp
        js.name = list(s["names"])
        js.mode = [0] * n
        js.position = s["position"]
        js.velocity = s["velocity"]
        js.effort = s["effort"]
        js.error_code = [0] * n
        self.pub_joint.publish(js)

        status = MotionControlStatus()
        status.header.stamp = stamp
        status.header.frame_id = "base_link"
        status.mode = MotionControlStatus.MODE_SERVO
        status.error_code = 0
        tf = TFMessage()
        for arm in ("left", "right"):
            e = s["ends"][arm]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = e["pos"]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = e["quat_xyzw"]
            w = Wrench()
            w.force.x, w.force.y, w.force.z = e["force"]
            w.torque.x, w.torque.y, w.torque.z = e["torque"]
            status.frame_names.append(END_FRAMES[arm])
            status.frame_poses.append(pose)
            status.frame_twists.append(Twist())
            status.frame_wrenchs.append(w)
            tf.transforms.append(self.transform(stamp, END_FRAMES[arm], e["pos"], e["quat_xyzw"]))
        self.pub_status.publish(status)
        tf.transforms.append(self.transform(stamp, "head_link3", s["head"]["pos"], s["head"]["quat_xyzw"]))
        self.pub_tf.publish(tf)

    @staticmethod
    def transform(stamp, child, p, q_xyzw):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "base_link"
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q_xyzw
        return t

    def publish_camera(self, header, blobs):
        stamp = self.node.get_clock().now().to_msg()        # 三张同一个时间戳(harness 按时间戳配对)
        jpg, depth, seg = blobs
        color = CompressedImage()
        color.header.stamp = stamp
        color.format = "jpeg"
        color.data = byte_array(jpg)
        self.pub_color.publish(color)
        self.pub_depth.publish(self.image_16u(stamp, header["width"], header["height"], depth))
        self.pub_seg.publish(self.image_16u(stamp, header["width"], header["height"], seg))

    @staticmethod
    def image_16u(stamp, width, height, data):
        img = Image()
        img.header.stamp = stamp
        img.width, img.height = width, height
        img.encoding = "16UC1"
        img.is_bigendian = 0
        img.step = width * 2
        img.data = byte_array(data)
        return img


def main():
    bridge = Bridge()
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
