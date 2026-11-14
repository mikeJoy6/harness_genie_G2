"""腕部相机:取一帧彩色(和深度,如果有)。2026-11-14 加,真机还没收到过画面。

现状(详见 docs/wrist_camera.md):
  · G2 上看得到 /gdk/camera/hand_{left,right}_color(sensor_msgs/CompressedImage),2026-11-13 实测 10s 收不到帧;
  · 没看到腕部深度话题。G01 的 GDK 指南里腕部是 D405(RGBD),v1.5.0 起才有 /camera/hand_*_depth,
    G2 是不是同一款、深度开没开,要用 scripts/check_wrist_camera.py 在真机上确认;
  · URDF 里没有腕部相机的 link,相机相对夹爪的位置不知道(没有手眼标定)。

用时才订、用完就撤(SensorNode),不在主节点上建删订阅。
"""
import threading
import time

import cv2
import numpy as np


def decode_color(msg):
    """CompressedImage(JPEG / PNG)→ BGR 图;解不开返回 None。"""
    return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)


def decode_depth(msg):
    """sensor_msgs/Image 深度 → 米为单位的 float32 图,0 = 没有深度。不认识的编码抛 ValueError。"""
    if msg.encoding in ("16UC1", "mono16"):
        raw = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return raw.astype(np.float32) / 1000.0
    if msg.encoding == "32FC1":
        raw = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return np.nan_to_num(raw.copy(), nan=0.0)
    raise ValueError(f"不认识的深度编码 {msg.encoding}")


class WristCamera:
    def __init__(self, sensor_node, cfg):
        """cfg = harness.yaml vision.wrist:{"timeout_s", "right": {"color": 话题, "depth": 话题或 null}, ...}"""
        self.sensor = sensor_node
        self.cfg = cfg
        self.lock = threading.Lock()
        self.got = {}

    def grab(self, arm):
        """取一帧。返回 {"ok", "bgr", "depth_m"(没配深度时 None), "topics"} 或 {"ok": False, "error"}。"""
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, Image

        topics = self.cfg.get(arm)
        if not topics:
            return {"ok": False, "error": f"没有配置 {arm} 手的腕部相机(harness.yaml vision.wrist)"}
        with self.lock:
            self.got = {}
        specs = [(CompressedImage, topics["color"], self._keep("color"), qos_profile_sensor_data)]
        if topics.get("depth"):
            specs.append((Image, topics["depth"], self._keep("depth"), qos_profile_sensor_data))
        wanted = len(specs)
        self.sensor.open(specs)
        try:
            deadline = time.monotonic() + self.cfg["timeout_s"]
            while time.monotonic() < deadline:
                with self.lock:
                    if len(self.got) == wanted:
                        break
                time.sleep(0.02)
        finally:
            self.sensor.close()
        with self.lock:
            got = dict(self.got)
        if "color" not in got:
            return {"ok": False, "error": f"{self.cfg['timeout_s']}s 内没收到 {topics['color']} 的画面"}
        bgr = decode_color(got["color"])
        if bgr is None:
            return {"ok": False, "error": f"{topics['color']} 的数据解不开(format={got['color'].format})"}
        depth = None
        if topics.get("depth"):
            if "depth" not in got:
                return {"ok": False, "error": f"彩色收到了,但 {self.cfg['timeout_s']}s 内没收到 {topics['depth']}"}
            depth = decode_depth(got["depth"])
        return {"ok": True, "bgr": bgr, "depth_m": depth, "topics": topics}

    def _keep(self, key):
        """回调:只留第一帧。"""
        def on_message(msg):
            with self.lock:
                if key not in self.got:
                    self.got[key] = msg
        return on_message
