"""查腕部相机:话题有哪些、收不收得到帧、是 RGB 还是 RGBD、分辨率帧率、有没有 TF 坐标系。只读,机器人不动。

    source ~/agi@g2_robot/env_g2.sh
    cd ~/robot_harness
    python3 scripts/check_wrist_camera.py                 # 默认听 3 秒
    python3 scripts/check_wrist_camera.py --seconds 10

收到的第一帧存到 photos/wrist_check/(已 gitignore;画面里可能有人,不要上传)。
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data

IMAGE_TYPES = ("sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo")
KEYWORDS = ("hand", "wrist")


class Collector:
    """记下收到的每一帧和收到的时刻。"""

    def __init__(self):
        self.received = []

    def on_message(self, msg):
        self.received.append((time.monotonic(), msg))


def describe(msg, kind):
    """一帧消息的一行说明。"""
    if kind.endswith("CompressedImage"):
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_UNCHANGED)
        if image is None:
            return f"format={msg.format},解不开"
        return f"format={msg.format},{image.shape[1]}x{image.shape[0]},通道 {1 if image.ndim == 2 else image.shape[2]}"
    if kind.endswith("CameraInfo"):
        return f"{msg.width}x{msg.height},fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}"
    text = f"encoding={msg.encoding},{msg.width}x{msg.height}"
    if msg.encoding in ("16UC1", "mono16"):
        depth = np.frombuffer(bytes(msg.data), np.uint16).astype(np.float32) / 1000.0
        valid = depth[depth > 0]
        if len(valid):
            text += f",有深度的像素 {len(valid) / depth.size:.0%},中位数 {np.median(valid):.3f}m(这是深度图)"
        else:
            text += ",全是 0"
    return text


def save(msg, kind, topic, folder):
    """把第一帧存下来看一眼。深度图按 0~1m 拉伸成灰度。"""
    name = topic.strip("/").replace("/", "_")
    if kind.endswith("CompressedImage"):
        path = folder / f"{name}.jpg"
        path.write_bytes(bytes(msg.data))
        return path
    if kind.endswith("Image") and msg.encoding in ("16UC1", "mono16"):
        depth = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        gray = np.clip(depth.astype(np.float32) / 1000.0, 0.0, 1.0) * 255
        path = folder / f"{name}_depth.png"
        cv2.imwrite(str(path), gray.astype(np.uint8))
        return path
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=3.0, help="每个话题听多久")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("check_wrist_camera")
    time.sleep(1.5)                                   # 等话题发现
    found = []
    for topic, types in node.get_topic_names_and_types():
        if any(k in topic.lower() for k in KEYWORDS):
            print(f"话题 {topic}  {types}")
            for kind in types:
                if kind in IMAGE_TYPES:
                    found.append((topic, kind))
    if not found:
        print("没有名字里带 hand / wrist 的图像话题")

    folder = Path("photos/wrist_check")
    folder.mkdir(parents=True, exist_ok=True)
    from rosidl_runtime_py.utilities import get_message
    results = {}
    for topic, kind in found:
        collector = Collector()
        sub = node.create_subscription(get_message(kind), topic, collector.on_message, qos_profile_sensor_data)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_subscription(sub)                # 这里是单线程 spin_once,不会和 executor 抢
        received = collector.received
        if not received:
            print(f"✗ {topic}:{args.seconds:.0f}s 收不到帧")
            results[topic] = None
            continue
        span = received[-1][0] - received[0][0]
        rate = (len(received) - 1) / span if span > 0 else 0.0
        msg = received[0][1]
        print(f"✓ {topic}:{len(received)} 帧,约 {rate:.1f}Hz,frame_id={msg.header.frame_id},{describe(msg, kind)}")
        path = save(msg, kind, topic, folder)
        if path is not None:
            print(f"    第一帧存在 {path}")
        results[topic] = kind

    from tf2_ros import Buffer, TransformListener
    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    frames = [f for f in buffer.all_frames_as_yaml().splitlines() if not f.startswith(" ")]
    cams = [f.rstrip(":") for f in frames if any(k in f.lower() for k in ("hand", "wrist", "camera", "optical"))]
    print("TF 里和相机 / 手有关的坐标系:", cams if cams else "没有")

    color = [t for t, k in results.items() if k and "color" in t]
    depth = [t for t, k in results.items() if k and "depth" in t]
    print()
    if depth:
        print(f"结论:有深度帧 → RGBD({depth})")
    elif color:
        print(f"结论:只收到彩色 → 目前只能当 RGB 用({color})")
    else:
        print("结论:腕部相机一帧都没收到。先查机器人端相机配置有没有打开腕部相机(见 docs/wrist_camera.md)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
