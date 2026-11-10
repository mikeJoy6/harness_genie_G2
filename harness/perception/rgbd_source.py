"""rgbd_source.py —— 头部 RGBD 帧来源（实时工具共用）：真机 ROS 话题，或 record_dataset 录的数据集。

每帧 = (彩色 BGR, 深度 uint16 mm, base_link→head_link3 4x4, 标题)。
真机：/gdk/camera/head_color + /gdk/camera/head_depth 按 header.stamp 精确配对（硬件同步，stamp 相同），
      头部位姿取 /gdk/tf 最新的 base_link → head_link3。
RosSource(sensor=SensorNode)：用处理器常驻的传感器节点（common/sensor_node.py），close() 停它的 executor 再撤订阅。
数据集：frames.jsonl 里每帧记下的头部位姿；a/d 翻页。
"""
import json
import threading
import time
from collections import OrderedDict

import cv2
import numpy as np

from .scene3d import pose_mat


class RosSource:
    def __init__(self, node_name=None, sensor=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, Image
        from tf2_msgs.msg import TFMessage

        self._rclpy = rclpy
        self.sensor = sensor
        self.lock = threading.Lock()
        self.colors = OrderedDict()
        self.pair, self.n, self.T = None, 0, None
        self.last_n = 0
        self.t_wait = time.monotonic()
        q = qos_profile_sensor_data
        specs = [(CompressedImage, "/gdk/camera/head_color", self._on_color, q),
                 (Image, "/gdk/camera/head_depth", self._on_depth, q),
                 (TFMessage, "/gdk/tf", self._on_tf, q)]
        if sensor is not None:
            sensor.open(specs)
            self.node = sensor.node
            return
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        for t, topic, cb, qos in specs:
            self.node.create_subscription(t, topic, cb, qos)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()

    @staticmethod
    def _stamp(m):
        return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec

    def _on_color(self, m):
        with self.lock:
            self.colors[self._stamp(m)] = bytes(m.data)
            while len(self.colors) > 30:
                self.colors.popitem(last=False)

    def _on_depth(self, m):
        if m.encoding != "16UC1":
            return
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width].copy()
        with self.lock:
            c = self.colors.pop(self._stamp(m), None)
            if c is not None:
                self.pair, self.n = (c, d), self.n + 1

    def _on_tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == "base_link" and t.child_frame_id == "head_link3":
                tr, r = t.transform.translation, t.transform.rotation
                with self.lock:
                    self.T = pose_mat(np.array([tr.x, tr.y, tr.z, r.x, r.y, r.z, r.w]))

    def next(self, key):
        """有新的一帧返回 (bgr, depth_mm, T, 标题)，否则 None。key 无用（数据集翻页用）。"""
        with self.lock:
            pair, n, T = self.pair, self.n, self.T
        if pair is None or T is None:
            if time.monotonic() - self.t_wait > 5.0:
                print(f"5s 还没齐：彩色+深度配对 {'有' if pair is not None else '无'}，/gdk/tf 头部位姿 "
                      f"{'有' if T is not None else '无'}。env_g2.sh source 了吗？网线千兆吗"
                      "（cat /sys/class/net/enp7s0/speed）？", flush=True)
                self.t_wait = time.monotonic()
            return None
        if n == self.last_n:
            return None
        self.last_n = n
        return cv2.imdecode(np.frombuffer(pair[0], np.uint8), cv2.IMREAD_COLOR), pair[1], T, "live"

    def close(self):
        if self.sensor is not None:
            self.sensor.close()
            return
        self.ex.shutdown()
        self.th.join(timeout=2.0)
        self.node.destroy_node()
        self._rclpy.try_shutdown()


class DatasetSource:
    def __init__(self, root):
        self.root = root
        frames = [json.loads(l) for l in (root / "frames.jsonl").read_text().splitlines() if l.strip()]
        self.frames = [f for f in frames if f.get("base_link_T_head_link3_xyz_qxyzw")]
        if not self.frames:
            raise SystemExit(f"{root}/frames.jsonl 里没有带头部位姿的帧")
        self.i = 0

    def next(self, key):
        """每次都返回当前帧（拖滑条时画面跟着变）；a/d/空格 翻页。"""
        if key in (ord("d"), ord(" ")):
            self.i = min(self.i + 1, len(self.frames) - 1)
        elif key == ord("a"):
            self.i = max(self.i - 1, 0)
        f = self.frames[self.i]
        img = cv2.imread(str(self.root / f["color"]))
        depth = cv2.imread(str(self.root / f["depth"]), cv2.IMREAD_UNCHANGED)
        T = pose_mat(np.array(f["base_link_T_head_link3_xyz_qxyzw"], float))
        return img, depth, T, f"[{self.i + 1}/{len(self.frames)}] {f['color']}"

    def close(self):
        pass
