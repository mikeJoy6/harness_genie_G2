"""仿真里代替 SAM 3:用仿真发的“真值分割图”给出物体掩码(完美掩码 → 完美坐标)。

仿真(sim/physics.py 渲染、sim/ros_bridge.py 发出)和彩色、深度同一时刻发一张 16 位分割图(/sim/camera/head_seg,每个像素是哪个物体,0 = 不是物体),
再发一张编号表(/sim/objects,JSON:[{"index", "name", "labels"}])。
segment() 返回的格式和 Sam3Client.segment 一样,所以 RosBackend.find_objects 后面那一段
(掩码 + 深度 → 可见表面中心的 base_link 坐标)原样复用:仿真深度没有误差,算出来就是完美坐标,
而且和真机 SAM 3 的坐标含义一样(可见表面的中位点,放在桌上的东西就是顶面)。

名字怎么匹配:query 和物体 labels 里任何一个相同就算(不分大小写,去掉 a / an / the,单复数都行)。
没匹配上就返回空,和 SAM 3 什么都没找到一样。
"""
import json
import threading
import time

import numpy as np

ARTICLES = ("a ", "an ", "the ")


def normalize(text):
    words = " ".join(str(text).lower().strip().split())
    for article in ARTICLES:
        if words.startswith(article):
            words = words[len(article):]
    return words


def singular(word):
    """很粗的单数化:够用来匹配 mice / phones / boxes / cups 这种。"""
    if word.endswith("mice"):
        return word[:-4] + "mouse"
    if word.endswith("people"):
        return word[:-6] + "person"
    if word.endswith("ies") and len(word) > 4:
        return word[:-3] + "y"
    if word.endswith("es") and word[:-2].endswith(("x", "s", "ch", "sh")):
        return word[:-2]
    if word.endswith("s") and not word.endswith("ss"):
        return word[:-1]
    return word


def label_matches(query, labels):
    q = normalize(query)
    for label in labels:
        lab = normalize(label)
        if q == lab or singular(q) == singular(lab):
            return True
    return False


def results_from_seg(seg, table, prompts):
    """seg:H×W 物体编号图;table:[{"index", "name", "labels"}];prompts:英文短语列表。
    返回 Sam3Client.segment 的格式:[{"prompt", "instances": [{"score", "box_xyxy", "area_px", "mask", "sim_name"}]}]。"""
    results = []
    for prompt in prompts:
        instances = []
        for obj in table:
            if not label_matches(prompt, obj["labels"]):
                continue
            mask = seg == obj["index"]
            area = int(mask.sum())
            if area == 0:                            # 匹配上了但这一帧看不见(挡住了 / 在视野外)
                continue
            vs, us = np.nonzero(mask)
            instances.append({"score": 1.0, "box_xyxy": [int(us.min()), int(vs.min()), int(us.max()), int(vs.max())],
                              "area_px": area, "mask": mask, "sim_name": obj["name"]})
        results.append({"prompt": prompt, "instances": instances})
    return results


class GroundTruthSegmenter:
    def __init__(self, node, objects_topic, timeout_s=5.0):
        """node:已经在 spin 的 rclpy 节点。订阅仿真的编号表(latched),timeout_s 内收不到就报错。"""
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.lock = threading.Lock()
        self.table = None
        self.objects_topic = objects_topic
        self.timeout_s = timeout_s
        self.last_timing_ms = None           # 和 Sam3Client 一样有这个属性(find_objects 的日志用)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = node.create_subscription(String, objects_topic, self._on_table, qos)

    def _on_table(self, msg):
        with self.lock:
            self.table = json.loads(msg.data)

    def objects(self):
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            with self.lock:
                if self.table is not None:
                    return self.table
            time.sleep(0.05)
        raise RuntimeError(f"{self.timeout_s}s 没收到仿真的物体编号表 {self.objects_topic}(仿真在跑吗?)")

    def segment(self, seg, prompts):
        if isinstance(prompts, str):
            prompts = [prompts]
        return results_from_seg(seg, self.objects(), prompts)
