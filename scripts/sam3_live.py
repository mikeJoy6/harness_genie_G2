"""sam3_live.py —— G2 头部相机实时画面 + SAM 3 分割。终端里随时输入要找的物体,画面上的掩码跟着换。

    # 终端 1:SAM 3 服务(Python 3.12)
    cd ~/sam3_service && .venv/bin/python server.py
    # 终端 2:本脚本(系统 Python 3.10 + ROS)
    source ~/agi@g2_robot/env_g2.sh
    python3 scripts/sam3_live.py "red block" "pen"
    # 然后在终端 2 输入:  green block, black bag   ← 回车即生效;空行 = 清空;q = 退出

只订阅相机,机器人不会动。

为什么要分线程(快输入、慢处理):
    相机 30 帧/秒,SAM 3 一帧约 0.35 秒。如果每帧都排队等分割,队列越堆越长,画面越来越旧。
    做法:相机和分割之间只放一个“最新帧格子”(LatestSlot)。新帧来了直接盖掉旧的,
    分割线程每次只拿当时最新的那一帧,中间的帧直接丢弃。

    ROS 回调线程   每来一帧:把消息对象放进 frames 格子。只换一个引用,不解码、不复制
          │
          ▼  frames(LatestSlot,永远只有最新一帧)
    分割线程       等到有比上次新的帧 → 把这帧 JPEG 字节原样发给 SAM 3 → 结果放进 results 格子
          │
          ▼  results(LatestSlot,永远只有最新一次结果)
    主线程         OpenCV 窗口:有新帧才解码一次,叠上最新掩码显示(cv2.imshow 必须在主线程)
    输入线程       读终端一行 → 换掉要找的短语(Prompts)

“零拷贝”指的是:G2 发的是 JPEG(CompressedImage),rclpy 收到后 msg.data 是一块 array.array 内存。
    回调里不碰它;分割线程把这块内存直接当 HTTP 请求体发出去(memoryview,不 base64、不复制);
    显示线程用 np.frombuffer 直接在这块内存上解码。整个过程中 JPEG 字节只有 rclpy 反序列化那一份。
"""
import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from harness.perception.sam3_client import Sam3Client, draw  # noqa: E402

TOPIC = "/gdk/camera/head_color"
WINDOW = "sam3_live"


class LatestSlot:
    """只放最新一个东西的格子。

    put():覆盖旧的(旧的直接丢,不排队),序号 seq 加 1。
    peek():看一眼现在的 (seq, 东西),不等待。
    wait_newer(seen_seq):等到 seq 比 seen_seq 新为止;超时返回 (seen_seq, None)。
    """

    def __init__(self):
        self.cond = threading.Condition()         # 一把锁 + “有新东西了”的通知
        self.seq = 0
        self.item = None

    def put(self, item):
        with self.cond:
            self.item = item
            self.seq += 1
            self.cond.notify_all()

    def peek(self):
        with self.cond:
            return self.seq, self.item

    def wait_newer(self, seen_seq, timeout_s):
        deadline = time.monotonic() + timeout_s
        with self.cond:
            while self.seq == seen_seq:
                left = deadline - time.monotonic()
                if left <= 0:
                    return seen_seq, None
                self.cond.wait(left)              # 睡着等 put() 叫醒,不空转
            return self.seq, self.item


class Prompts:
    """当前要找的短语。每改一次 version 加 1,用来扔掉“按旧短语算出来”的结果。"""

    def __init__(self, words):
        self.lock = threading.Lock()
        self.words = list(words)
        self.version = 0

    def set(self, words):
        with self.lock:
            self.words = list(words)
            self.version += 1

    def get(self):
        with self.lock:
            return self.version, list(self.words)


class CameraSubscriber:
    """订阅头部彩色 JPEG,回调里只做 frames.put((msg, 收到的时间))。"""

    def __init__(self, topic, frames):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage

        self.rclpy = rclpy
        self.frames = frames
        rclpy.init()
        self.node = rclpy.create_node("sam3_live")
        # depth=1:ROS 自己的接收队列也只留 1 帧,回调来不及时不会在 DDS 里堆旧图
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(CompressedImage, topic, self.on_image, qos)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_image(self, msg):
        self.frames.put((msg, time.monotonic()))  # 只存引用:不解码、不复制

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        self.rclpy.try_shutdown()


class Stats:
    """分割线程写、主线程读的几个数字。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.segmented = 0          # 分割了多少帧
        self.skipped = 0            # 分割线程跳过(没来得及处理就被新帧盖掉)的帧数
        self.error = ""

    def add(self, segmented=0, skipped=0, error=None):
        with self.lock:
            self.segmented += segmented
            self.skipped += skipped
            if error is not None:
                self.error = error

    def get(self):
        with self.lock:
            return self.segmented, self.skipped, self.error


def segment_loop(frames, prompts, results, client, threshold, stop, stats):
    """分割线程:永远只处理当时最新的一帧。"""
    seen = 0
    while not stop.is_set():
        seq, item = frames.wait_newer(seen, timeout_s=0.5)
        if item is None:                          # 0.5 s 没有新帧,回去看一眼要不要退出
            continue
        if seen > 0:
            stats.add(skipped=seq - seen - 1)     # seq 跳了几个,就是有几帧被新帧盖掉了
        seen = seq
        msg, t_recv = item
        version, words = prompts.get()
        if not words:
            results.put({"version": version, "results": [], "msg": msg, "latency_ms": 0.0})
            continue
        t0 = time.monotonic()
        try:
            out = client.segment_jpeg(msg.data, words, threshold)   # msg.data 原样发出,不复制
        except RuntimeError as e:
            stats.add(error=str(e))
            stop.wait(1.0)                        # 服务没开或出错:等 1 s 再试,不刷屏
            continue
        stats.add(segmented=1, error="")
        results.put({"version": version, "results": out, "msg": msg,
                     "infer_ms": (time.monotonic() - t0) * 1000,
                     "latency_ms": (time.monotonic() - t_recv) * 1000})


def split_words(line):
    """'red block, pen' 或 'red block,pen'(中英文逗号都行)→ ['red block', 'pen']"""
    words = []
    for part in line.replace("，", ",").split(","):
        part = part.strip()
        if part:
            words.append(part)
    return words


def input_loop(prompts, stop):
    """输入线程:input() 会一直阻塞等回车,所以单独放一个线程,不卡画面。"""
    print("输入要找的物体(英文,逗号分隔),回车生效;空行清空;q 退出。", flush=True)
    while not stop.is_set():
        try:
            line = input()
        except EOFError:
            break
        line = line.strip()
        if line.lower() in ("q", "quit", "exit"):
            stop.set()
            break
        words = split_words(line)
        prompts.set(words)
        print(f"现在找:{words if words else '(什么都不找)'}", flush=True)


def decode(msg):
    """在 ROS 消息那块内存上直接解码(np.frombuffer 不复制)。"""
    return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)


def put_lines(img, lines):
    y = 20
    for text in lines:
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        y += 20


def main():
    parser = argparse.ArgumentParser(description="G2 头部相机 + SAM 3 实时分割")
    parser.add_argument("prompts", nargs="*", help="一开始要找的物体,英文短语,如 \"red block\"")
    parser.add_argument("--topic", default=TOPIC, help="sensor_msgs/CompressedImage 话题")
    parser.add_argument("--url", default="http://127.0.0.1:8765", help="SAM 3 服务地址")
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--sync", action="store_true",
                        help="显示“被分割的那一帧”而不是最新帧:掩码和画面严格对齐,但画面会慢约 0.35 s")
    args = parser.parse_args()

    client = Sam3Client(args.url, timeout_s=10.0)
    try:
        print("SAM 3 服务:", client.health(), flush=True)
    except RuntimeError as e:
        raise SystemExit(str(e))

    frames, results = LatestSlot(), LatestSlot()
    prompts = Prompts(args.prompts)
    stats = Stats()
    stop = threading.Event()
    camera = CameraSubscriber(args.topic, frames)
    worker = threading.Thread(target=segment_loop, daemon=True,
                              args=(frames, prompts, results, client, args.threshold, stop, stats))
    worker.start()
    threading.Thread(target=input_loop, args=(prompts, stop), daemon=True).start()
    print(f"现在找:{args.prompts if args.prompts else '(什么都不找)'}   窗口里按 q 或 Esc 也能退出", flush=True)

    shown_seq, shown_img = 0, None
    fps_count, fps_t0, cam_fps, seg_fps, seg_done0 = 0, time.monotonic(), 0.0, 0.0, 0
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    try:
        while not stop.is_set():
            # 1. 有新帧才解码(“指针指向最新,但不真取图”:没人要看时不花这份力气)
            seq, item = frames.peek()
            if item is not None and seq != shown_seq:
                img = decode(item[0])
                if img is not None:
                    fps_count += seq - shown_seq if shown_seq else 1
                    shown_seq, shown_img = seq, img

            # 2. 取最新结果;短语改过之后,旧短语的结果不画
            version, words = prompts.get()
            rseq, res = results.peek()
            if res is not None and res["version"] != version:
                res = None

            # 3. 每秒更新一次帧率
            now = time.monotonic()
            if now - fps_t0 >= 1.0:
                done, _, _ = stats.get()
                cam_fps = fps_count / (now - fps_t0)
                seg_fps = (done - seg_done0) / (now - fps_t0)
                fps_count, fps_t0, seg_done0 = 0, now, done

            if shown_img is None:
                view = np.zeros((400, 640, 3), np.uint8)
                put_lines(view, [f"waiting for {args.topic} ..."])
            else:
                base = shown_img
                if args.sync and res is not None:
                    base = decode(res["msg"])
                view = draw(base, res["results"]) if res is not None else base.copy()
                done, skipped, error = stats.get()
                lines = [f"find: {', '.join(words) if words else '(nothing)'}",
                         f"camera {cam_fps:4.1f} fps   sam3 {seg_fps:3.1f} fps   skipped {skipped}"]
                if res is not None and res.get("infer_ms"):
                    lines.append(f"infer {res['infer_ms']:.0f} ms   mask age {res['latency_ms']:.0f} ms"
                                 f"{'  (sync)' if args.sync else ''}")
                if error:
                    lines.append("ERROR: " + error[:70])
                put_lines(view, lines)
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        stop.set()
        worker.join(timeout=3.0)
        camera.close()
        cv2.destroyAllWindows()
        done, skipped, _ = stats.get()
        print(f"分割了 {done} 帧,跳过旧帧 {skipped} 帧。")


if __name__ == "__main__":
    main()
