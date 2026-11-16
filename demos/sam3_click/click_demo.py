"""G2 头部相机实时画面 + SAM 3 交互分割演示:在画面上点、拉框、打字,看 SAM 3 怎么补全物体。

纯演示,不接 harness,机器人不会动(只订阅相机)。

    # 终端 1:交互服务(Python 3.12;常规 SAM 3 服务 8765 先停掉,省显存)
    cd ~/robot_harness
    ~/sam3_service/.venv/bin/python demos/sam3_click/server.py
    # 终端 2:本脚本(系统 Python 3.10 + ROS)
    source ~/agi@g2_robot/env_g2.sh
    cd ~/robot_harness
    python3 demos/sam3_click/click_demo.py
    python3 demos/sam3_click/click_demo.py --image traces/sam3/<某次>/color.jpg   # 不连机器人,用存好的图

操作(窗口要有焦点;文本模式在终端 2 打字):
    空格     冻结 / 解冻画面。所有分割都在冻结的那一帧上做(编码一次,之后每次点击只要几十 ms)
    1 点     左键 = 物体上的点(绿),右键 = 不是物体的点(红);只有一个点时 Tab 切换 3 个候选
             n / 回车 = 留下这个物体,开始下一个;z = 撤销上一个点
    2 框     拖一个框 → 补全框里那一个物体
    3 示例框 拖一个框 → 找出画面里所有和它同类的东西
    4 文本   在终端输入英文名(逗号分隔)→ 找出所有这种东西
    5 全分割 网格撒点、每个点补全一个、去重 → 不要名字,画面里能分出来的都分
    c 清空   s 存图到 photos/sam3_click/(本机,已 gitignore)   q / Esc 退出
"""
import argparse
import base64
import json
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

WINDOW = "sam3_click"
MODES = {ord("1"): "point", ord("2"): "box", ord("3"): "exemplar", ord("4"): "text", ord("5"): "everything"}
MODE_HELP = {
    "point": "point: L=object R=not-object  Tab=next candidate  n=keep  z=undo",
    "box": "box: drag a box -> that one object",
    "exemplar": "exemplar: drag a box -> everything like it",
    "text": "text: type English names in the terminal",
    "everything": "everything: press 5 again to rerun",
}


# ---------------- 和服务通信 ----------------
class Client:
    def __init__(self, url, timeout_s=60.0):
        self.url = url.rstrip("/")
        self.timeout_s = timeout_s

    def _send(self, path, data, content_type):
        request = urllib.request.Request(self.url + path, data=data, headers={"Content-Type": content_type})
        try:
            response = urllib.request.urlopen(request, timeout=self.timeout_s)
            reply = json.load(response)
        except urllib.error.HTTPError as e:
            reply = json.load(e)
        except urllib.error.URLError as e:
            raise RuntimeError(f"连不上 {self.url}(server.py 开了吗?):{e.reason}")
        if not reply.get("ok"):
            raise RuntimeError(reply.get("error", "服务报错"))
        return reply

    def post_json(self, path, body):
        return self._send(path, json.dumps(body).encode("utf-8"), "application/json")

    def post_jpeg(self, jpeg):
        return self._send("/image", bytes(jpeg), "image/jpeg")

    def health(self):
        response = urllib.request.urlopen(self.url + "/health", timeout=3.0)
        return json.load(response)


def decode_mask(inst):
    png = np.frombuffer(base64.b64decode(inst["mask_png_b64"]), np.uint8)
    return cv2.imdecode(png, cv2.IMREAD_GRAYSCALE) > 0


# ---------------- 相机 ----------------
class Camera:
    """订阅头部彩色 JPEG,只留最新一帧。"""

    def __init__(self, topic):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage

        self.rclpy = rclpy
        self.lock = threading.Lock()
        self.latest = None
        rclpy.init()
        self.node = rclpy.create_node("sam3_click_demo")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(CompressedImage, topic, self.on_image, qos)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_image(self, msg):
        with self.lock:
            self.latest = bytes(msg.data)

    def get(self):
        with self.lock:
            return self.latest

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        self.rclpy.try_shutdown()


class StillImage:
    """--image:用一张存好的 JPEG 代替相机(不连机器人也能演示)。"""

    def __init__(self, path):
        self.jpeg = Path(path).read_bytes()

    def get(self):
        return self.jpeg

    def close(self):
        pass


# ---------------- 后台干活(请求不卡画面) ----------------
class Worker:
    """一次只做一件事;做的时候又来新任务,只留最新的那个(连点时不排长队)。"""

    def __init__(self, client):
        self.client = client
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.job = None              # (名字, 参数字典)
        self.result = None           # {"job", "instances": [(掩码, 分数, 标签)], "ms", "error", "image_id"}
        self.busy = ""
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def submit(self, name, args):
        with self.lock:
            self.job = (name, args)
        self.wake.set()

    def take_result(self):
        with self.lock:
            result = self.result
            self.result = None
            return result

    def loop(self):
        while not self.stop.is_set():
            self.wake.wait(0.2)
            self.wake.clear()
            with self.lock:
                job = self.job
                self.job = None
            if job is None:
                continue
            name, args = job
            self.busy = name
            t0 = time.monotonic()
            result = {"job": name, "instances": [], "error": "", "image_id": None, "args": args}
            try:
                result.update(self.run(name, args))
            except RuntimeError as e:
                result["error"] = str(e)
            result["ms"] = (time.monotonic() - t0) * 1000
            self.busy = ""
            with self.lock:
                self.result = result

    def run(self, name, args):
        c = self.client
        if name == "image":
            reply = c.post_jpeg(args["jpeg"])
            return {"image_id": reply["image_id"]}
        if name == "points":
            reply = c.post_json("/points", {"image_id": args["image_id"], "points": args["points"],
                                            "box": args.get("box")})
            return {"instances": unpack(reply["instances"], "")}
        if name == "exemplar":
            reply = c.post_json("/exemplar", {"image_id": args["image_id"], "box": args["box"]})
            return {"instances": unpack(reply["instances"], "like box")}
        if name == "text":
            reply = c.post_json("/text", {"image_id": args["image_id"], "prompts": args["prompts"]})
            found = []
            for r in reply["results"]:
                found.extend(unpack(r["instances"], r["prompt"]))
            return {"instances": found}
        if name == "everything":
            reply = c.post_json("/everything", {"image_id": args["image_id"], "grid": args["grid"],
                                                "max_area_frac": args["max_area_frac"]})
            return {"instances": unpack(reply["instances"], "")}
        raise RuntimeError(f"不认识的任务 {name}")


def unpack(instances, label):
    found = []
    for inst in instances:
        found.append((decode_mask(inst), inst["score"], label))
    return found


# ---------------- 终端输入(文本模式) ----------------
class TextInput:
    def __init__(self):
        self.lock = threading.Lock()
        self.pending = None
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while True:
            try:
                line = input()
            except EOFError:
                return
            words = []
            for part in line.replace("，", ",").split(","):
                if part.strip():
                    words.append(part.strip())
            with self.lock:
                self.pending = words

    def take(self):
        with self.lock:
            words = self.pending
            self.pending = None
            return words


# ---------------- 界面状态 ----------------
class Demo:
    def __init__(self, args):
        self.args = args
        self.scale = args.scale
        self.mode = "point"
        self.frozen = None           # 冻结的那帧(BGR)
        self.frozen_jpeg = None
        self.image_id = None
        self.points = []             # 当前物体的点 [[x, y, 1 或 0], ...](原图像素)
        self.candidates = []         # 当前物体的候选 [(掩码, 分数, 标签)]
        self.pick = 0                # 用第几个候选
        self.kept = []               # 已留下的物体 [(掩码, 分数, 标签)]
        self.found = []              # 示例框 / 文本 / 全分割的结果
        self.drag_start = None
        self.drag_now = None
        self.status = "space = freeze a frame"
        self.colors = np.random.default_rng(7).integers(60, 255, size=(256, 3))

    # 鼠标:窗口坐标 → 原图坐标
    def on_mouse(self, event, x, y, flags, param):
        if self.frozen is None or self.image_id is None:
            return
        px, py = int(x / self.scale), int(y / self.scale)
        if self.mode == "point":
            if event == cv2.EVENT_LBUTTONDOWN:
                self.points.append([px, py, 1])
                self.send_points()
            elif event == cv2.EVENT_RBUTTONDOWN:
                self.points.append([px, py, 0])
                self.send_points()
        elif self.mode in ("box", "exemplar"):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drag_start = (px, py)
                self.drag_now = (px, py)
            elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
                self.drag_now = (px, py)
            elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
                x0, y0 = self.drag_start
                box = [min(x0, px), min(y0, py), max(x0, px), max(y0, py)]
                self.drag_start = None
                if box[2] - box[0] < 4 or box[3] - box[1] < 4:
                    self.status = "box too small"
                    return
                if self.mode == "box":
                    self.worker.submit("points", {"image_id": self.image_id, "points": None, "box": box})
                else:
                    self.worker.submit("exemplar", {"image_id": self.image_id, "box": box})

    def send_points(self):
        self.pick = 0
        self.worker.submit("points", {"image_id": self.image_id, "points": list(self.points)})

    def freeze(self, jpeg):
        self.frozen_jpeg = jpeg
        self.frozen = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.image_id = None
        self.clear()
        self.status = "encoding frame ..."
        self.worker.submit("image", {"jpeg": jpeg})

    def unfreeze(self):
        self.frozen = None
        self.image_id = None
        self.clear()
        self.status = "live"

    def clear(self):
        self.points = []
        self.candidates = []
        self.kept = []
        self.found = []

    def keep_current(self):
        if self.candidates:
            self.kept.append(self.candidates[self.pick])
        self.points = []
        self.candidates = []

    def handle_result(self, result):
        if result["error"]:
            self.status = "ERROR: " + result["error"][:80]
            return
        job = result["job"]
        if job == "image":
            self.image_id = result["image_id"]
            self.status = f"frozen, encoded in {result['ms']:.0f} ms"
            return
        if job == "points" and result["args"].get("points"):
            self.candidates = result["instances"]
            self.pick = 0
        elif job == "points":                  # 框:直接留下
            self.kept.extend(result["instances"][:1])
        else:
            self.found = result["instances"]
        self.status = f"{job}: {len(result['instances'])} mask(s) in {result['ms']:.0f} ms"

    def draw(self, base):
        view = base.copy()
        layers = []
        for i, item in enumerate(self.kept):
            layers.append((item, self.colors[i % 256], f"#{i + 1}"))
        for i, item in enumerate(self.found):
            layers.append((item, self.colors[(i + 100) % 256], item[2]))
        if self.candidates:
            layers.append((self.candidates[self.pick], np.array([255, 255, 0]), "current"))
        for (mask, score, label), color, name in layers:
            view[mask] = (0.45 * view[mask] + 0.55 * color).astype(np.uint8)
        view = cv2.resize(view, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_LINEAR)
        for (mask, score, label), color, name in layers:
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            scaled = []
            for c in contours:
                scaled.append((c * self.scale).astype(np.int32))
            cv2.drawContours(view, scaled, -1, (255, 255, 255), 1)
            ys, xs = np.nonzero(mask)
            if len(xs):
                text = f"{name} {score:.2f}" if name == label or not label else f"{label} {score:.2f}"
                put_text(view, text, int(xs.mean() * self.scale), int(ys.mean() * self.scale), 0.45)
        for x, y, positive in self.points:
            color = (0, 220, 0) if positive else (0, 0, 255)
            cv2.circle(view, (int(x * self.scale), int(y * self.scale)), 6, color, -1)
            cv2.circle(view, (int(x * self.scale), int(y * self.scale)), 6, (255, 255, 255), 1)
        if self.drag_start is not None and self.drag_now is not None:
            p0 = (int(self.drag_start[0] * self.scale), int(self.drag_start[1] * self.scale))
            p1 = (int(self.drag_now[0] * self.scale), int(self.drag_now[1] * self.scale))
            cv2.rectangle(view, p0, p1, (0, 255, 255), 2)
        return view


def put_text(img, text, x, y, size=0.5):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, (255, 255, 255), 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="G2 头部相机 + SAM 3 交互分割演示")
    parser.add_argument("--topic", default="/gdk/camera/head_color", help="sensor_msgs/CompressedImage 话题")
    parser.add_argument("--url", default="http://127.0.0.1:8766", help="demos/sam3_click/server.py 的地址")
    parser.add_argument("--scale", type=float, default=2.0, help="窗口放大倍数(头部彩色只有 640x400)")
    parser.add_argument("--grid", type=int, default=16, help="全分割每行撒几个点")
    parser.add_argument("--max-area", type=float, default=0.15, help="全分割时比画面这么大比例还大的块不要(桌面 / 地面)")
    parser.add_argument("--image", help="不用相机,用这张 JPEG(不需要 ROS)")
    args = parser.parse_args()

    client = Client(args.url)
    try:
        print("服务:", client.health(), flush=True)
    except (urllib.error.URLError, OSError) as e:
        raise SystemExit(f"连不上 {args.url}:{e}(先开 demos/sam3_click/server.py)")

    demo = Demo(args)
    demo.worker = Worker(client)
    text_input = TextInput()
    if args.image:
        camera = StillImage(args.image)
    else:
        camera = Camera(args.topic)
    print(__doc__.split("操作")[1], flush=True)
    save_dir = Path(__file__).resolve().parents[2] / "photos" / "sam3_click"
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, demo.on_mouse)
    try:
        while True:
            result = demo.worker.take_result()
            if result is not None:
                demo.handle_result(result)
            words = text_input.take()
            if words:
                demo.mode = "text"
                if demo.image_id is None:
                    demo.status = "freeze a frame first (space), then type again"
                else:
                    demo.worker.submit("text", {"image_id": demo.image_id, "prompts": words})

            if demo.frozen is not None:
                view = demo.draw(demo.frozen)
            else:
                jpeg = camera.get()
                if jpeg is None:
                    view = np.zeros((int(400 * args.scale), int(640 * args.scale), 3), np.uint8)
                    put_text(view, f"waiting for {args.topic} ...", 10, 30)
                else:
                    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    view = cv2.resize(frame, None, fx=args.scale, fy=args.scale)
            busy = f"  [{demo.worker.busy} ...]" if demo.worker.busy else ""
            put_text(view, f"[{demo.mode}] {MODE_HELP[demo.mode]}", 10, 22)
            put_text(view, f"{'FROZEN' if demo.frozen is not None else 'LIVE'}  {demo.status}{busy}", 10, 44)
            put_text(view, "space freeze | 1 point 2 box 3 exemplar 4 text 5 everything | c clear s save q quit",
                     10, view.shape[0] - 12, 0.45)
            cv2.imshow(WINDOW, view)

            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                if demo.frozen is None:
                    jpeg = camera.get()
                    if jpeg is not None:
                        demo.freeze(jpeg)
                else:
                    demo.unfreeze()
            elif key in MODES:
                demo.mode = MODES[key]
                demo.points = []
                demo.candidates = []
                if demo.mode == "everything" and demo.image_id is not None:
                    demo.worker.submit("everything", {"image_id": demo.image_id, "grid": args.grid,
                                                      "max_area_frac": args.max_area})
                if demo.mode == "text":
                    print("在这个终端输入英文名(逗号分隔),回车:", flush=True)
            elif key == 9 and demo.candidates:               # Tab
                demo.pick = (demo.pick + 1) % len(demo.candidates)
            elif key in (ord("n"), 13):
                demo.keep_current()
            elif key == ord("z") and demo.points:
                demo.points.pop()
                if demo.points:
                    demo.send_points()
                else:
                    demo.candidates = []
            elif key == ord("c"):
                demo.clear()
            elif key == ord("s"):
                save_dir.mkdir(parents=True, exist_ok=True)
                path = save_dir / f"{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(str(path), view)
                demo.status = f"saved {path.name}"
    finally:
        demo.worker.stop.set()
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
