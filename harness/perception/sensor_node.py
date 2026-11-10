"""sensor_node.py —— 处理器里"用时才订、用完就退"的大流量订阅（头部相机、雷达）。

不能在主节点上边 spin 边 destroy_subscription：Humble 的 MultiThreadedExecutor 正在 take 那个订阅时它被销毁，
executor 抛 InvalidHandle，整个处理器崩（2026-11-03：approach_board 失败退出时撤雷达订阅，processor 直接退出）。
所以每类传感器一个独立节点（处理器启动时建，常驻，一个 DDS participant）+ 每次 open 一个独立 executor：
close 先停 executor、等线程退出，再撤订阅——撤的时候没人在 spin，不会抢。
"""
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor


class SensorNode:
    def __init__(self, name):
        self.node = rclpy.create_node(name)
        self.lock = threading.Lock()
        self.ex = self.th = None
        self.subs = []

    def open(self, specs):
        """specs：[(消息类型, 话题, 回调, qos), ...]。已开着就先关再开（重连用）。"""
        with self.lock:
            self._close()
            self.subs = [self.node.create_subscription(t, topic, cb, qos) for t, topic, cb, qos in specs]
            self.ex = SingleThreadedExecutor()
            self.ex.add_node(self.node)
            self.th = threading.Thread(target=self._spin, args=(self.ex,), daemon=True)
            self.th.start()

    @staticmethod
    def _spin(ex):
        try:
            ex.spin()
        except Exception:                      # executor 关闭时的竞争，不影响处理器
            pass

    def close(self):
        with self.lock:
            self._close()

    def _close(self):
        if self.ex is not None:
            self.ex.shutdown(timeout_sec=2.0)
            self.th.join(timeout=2.0)
            self.ex = self.th = None
        for s in self.subs:
            self.node.destroy_subscription(s)
        self.subs = []

    def destroy(self):
        self.close()
        self.node.destroy_node()
