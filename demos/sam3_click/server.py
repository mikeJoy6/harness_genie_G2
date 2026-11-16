"""SAM 3 交互演示服务:在 SAM 3 的基础上打开“点 / 框补全”(SAM 1 那种交互分割),给 click_demo.py 用。

纯演示,不接 harness。和 ~/sam3_service/server.py 是两个独立的服务(那个只有文本分割,harness 在用)。
要用 sam3 的 Python 3.12 环境跑:

    cd ~/robot_harness
    ~/sam3_service/.venv/bin/python demos/sam3_click/server.py          # 默认 127.0.0.1:8766

显存:常规服务常驻约 3.4 GB,这个多一个交互头,两个一起开 8 GB 卡可能不够 —— 先把常规服务(8765)停掉。

为什么先“设图”再“点”:
    图像编码最慢(几百 ms),点 / 框补全很快(几十 ms)。所以先 POST /image 把一帧编码好、存起来,
    拿到 image_id;之后每次点击只带 image_id + 点,不用重新编码。

接口(JSON 回复;掩码都是 PNG base64,和常规服务一样):
    GET  /health
    POST /image       请求体 = JPEG 原始字节(Content-Type: image/jpeg)  → {"image_id", "width", "height"}
    POST /points      {"image_id", "points": [[x, y, 1 或 0], ...], "box": [x0, y0, x1, y1] 或 null}
                      1 = 点在物体上,0 = 点不在物体上;只给一个点时返回 3 个候选(小部件 / 整个物体 / 更大一片)
    POST /text        {"image_id", "prompts": ["mouse", ...], "threshold": 0.5}   按名字找所有实例
    POST /exemplar    {"image_id", "box": [x0, y0, x1, y1], "threshold": 0.5}    找所有“和框里这个同类”的
    POST /everything  {"image_id", "grid": 16, "min_score": 0.85, "max_area_frac": 0.3}
                      网格撒点、每个点补全一个物体、去重 —— 不要名字的“全分割”
"""
import argparse
import base64
import io
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 和 ~/sam3_service/server.py 一样:huggingface_hub 不认 socks:// 代理写法
for key in ("all_proxy", "ALL_PROXY"):
    if os.environ.get(key, "").startswith("socks://"):
        os.environ[key] = "http://" + os.environ[key][len("socks://"):]

import numpy as np
import torch
from huggingface_hub import try_to_load_from_cache
from PIL import Image

MAX_BODY_BYTES = 20 * 1024 * 1024
KEEP_IMAGES = 3              # 最多缓存几张编码好的图(每张几百 MB 显存的特征,别多存)


def encode_mask(mask):
    """布尔掩码 → PNG → base64。"""
    buf = io.BytesIO()
    Image.fromarray(mask.astype(np.uint8) * 255).save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


def mask_box(mask):
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return [0, 0, 0, 0]
    return [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]


def mask_iou(a, b):
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    if union == 0:
        return 0.0
    return float(inter) / float(union)


def instance(mask, score):
    return {"score": round(float(score), 4), "box_xyxy": mask_box(mask), "area_px": int(mask.sum()),
            "mask_png_b64": encode_mask(mask)}


class Interactive:
    def __init__(self, threshold, bf16):
        from sam3.model.sam3_image_processor import Sam3Processor
        from sam3.model_builder import build_sam3_image_model

        t0 = time.monotonic()
        cached = try_to_load_from_cache("facebook/sam3", "sam3.pt")
        if isinstance(cached, str):
            print(f"[sam3-click] 用本机权重 {cached}", flush=True)
            self.model = build_sam3_image_model(checkpoint_path=cached, enable_inst_interactivity=True)
        else:
            print("[sam3-click] 本机没有权重,从 Hugging Face 下载(要先 hf auth login)", flush=True)
            self.model = build_sam3_image_model(enable_inst_interactivity=True)
        self.processor = Sam3Processor(self.model, confidence_threshold=threshold)
        self.load_s = time.monotonic() - t0
        self.default_threshold = threshold
        self.bf16 = bf16 and torch.cuda.is_available()
        self.lock = threading.Lock()          # 只有一块 GPU:一个一个来
        self.images = {}                      # image_id → (state, 宽, 高)
        self.order = []
        self.next_id = 1

    def autocast(self):
        return torch.autocast("cuda", dtype=torch.bfloat16, enabled=self.bf16)

    def gpu_mem_mb(self):
        if not torch.cuda.is_available():
            return None
        return {"allocated": round(torch.cuda.memory_allocated() / 2**20),
                "peak": round(torch.cuda.max_memory_allocated() / 2**20)}

    # ---- 设图 ----
    def set_image(self, jpeg):
        image = Image.open(io.BytesIO(jpeg)).convert("RGB")
        with self.lock:
            with self.autocast():
                state = self.processor.set_image(image)
            image_id = self.next_id
            self.next_id += 1
            self.images[image_id] = (state, image.width, image.height)
            self.order.append(image_id)
            while len(self.order) > KEEP_IMAGES:
                old = self.order.pop(0)
                del self.images[old]
        return image_id, image.width, image.height

    def _state(self, image_id):
        got = self.images.get(int(image_id))
        if got is None:
            raise KeyError(f"image_id {image_id} 不在缓存里(最多留 {KEEP_IMAGES} 张),请重新 POST /image")
        return got

    # ---- 点 / 框补全(SAM 1 式,补全“这一个”物体) ----
    def points(self, image_id, points, box):
        state, _, _ = self._state(image_id)
        coords = None
        labels = None
        if points:
            coords = np.array([[p[0], p[1]] for p in points], dtype=np.float32)
            labels = np.array([int(p[2]) for p in points], dtype=np.int32)
        box_np = None if box is None else np.array(box, dtype=np.float32)
        multimask = points is not None and len(points) == 1 and box is None   # 一个点有歧义:给 3 个候选
        with self.lock:
            with self.autocast():
                masks, scores, _ = self.model.predict_inst(state, point_coords=coords, point_labels=labels,
                                                           box=box_np, multimask_output=multimask)
        order = np.argsort(-scores)
        return [instance(masks[i] > 0.5, scores[i]) for i in order]

    # ---- 按名字找所有实例(SAM 3 概念分割,和常规服务一样) ----
    def text(self, image_id, prompts, threshold):
        state, _, _ = self._state(image_id)
        results = []
        with self.lock:
            with self.autocast():
                self.processor.confidence_threshold = threshold
                for prompt in prompts:
                    out = self.processor.set_text_prompt(prompt=prompt, state=state)
                    results.append((prompt, out["masks"].squeeze(1).cpu().numpy(),
                                    out["scores"].float().cpu().numpy()))
                self.processor.reset_all_prompts(state)
        reply = []
        for prompt, masks, scores in results:
            order = np.argsort(-scores)
            reply.append({"prompt": prompt, "instances": [instance(masks[i], scores[i]) for i in order]})
        return reply

    # ---- 示例框:找所有和框里同类的 ----
    def exemplar(self, image_id, box, threshold):
        state, width, height = self._state(image_id)
        x0, y0, x1, y1 = box
        cxcywh = [(x0 + x1) / 2 / width, (y0 + y1) / 2 / height, abs(x1 - x0) / width, abs(y1 - y0) / height]
        with self.lock:
            with self.autocast():
                self.processor.confidence_threshold = threshold
                out = self.processor.add_geometric_prompt(box=cxcywh, label=True, state=state)
                masks = out["masks"].squeeze(1).cpu().numpy()
                scores = out["scores"].float().cpu().numpy()
                self.processor.reset_all_prompts(state)
        order = np.argsort(-scores)
        return [instance(masks[i], scores[i]) for i in order]

    # ---- 全分割:网格撒点,每个点补全一个,去重 ----
    def everything(self, image_id, grid, min_score, max_area_frac):
        state, width, height = self._state(image_id)
        rows = max(1, round(grid * height / width))
        grid_points = []
        for row in range(rows):
            for col in range(grid):
                grid_points.append([(col + 0.5) * width / grid, (row + 0.5) * height / rows])
        candidates = []
        for start in range(0, len(grid_points), 32):           # 32 个点一批:每个点各当一个物体
            chunk = np.array(grid_points[start:start + 32], np.float32)
            coords = chunk[:, None, :]                          # (B, 1, 2):B 个物体,每个 1 个点
            labels = np.ones((len(chunk), 1), np.int32)
            with self.lock:
                with self.autocast():
                    masks, scores, _ = self.model.predict_inst(state, point_coords=coords, point_labels=labels,
                                                               multimask_output=True)
            if masks.ndim == 3:                                 # 只有 1 个点时批维度被 squeeze 掉了
                masks = masks[None]
                scores = scores[None]
            for one_masks, one_scores in zip(masks, scores):
                best = int(np.argmax(one_scores))
                mask = one_masks[best] > 0.5
                area = int(mask.sum())
                if one_scores[best] < min_score or area < 30 or area > max_area_frac * width * height:
                    continue                      # 太不确定、太碎、太大(桌面 / 墙)都不要
                candidates.append((float(one_scores[best]), mask))
        candidates.sort(key=score_of, reverse=True)
        kept = []
        for score, mask in candidates:            # 和已经留下的重叠太多就是同一个东西
            duplicate = False
            for _, other in kept:
                if mask_iou(mask, other) > 0.6:
                    duplicate = True
                    break
            if not duplicate:
                kept.append((score, mask))
        return [instance(mask, score) for score, mask in kept], len(grid_points)


def score_of(item):
    return item[0]


def make_handler(seg):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code, body):
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path != "/health":
                return self._reply(404, {"ok": False, "error": "GET 只有 /health"})
            self._reply(200, {"ok": True, "model": "sam3+interactive", "load_s": round(seg.load_s, 1),
                              "bf16": seg.bf16, "gpu_mem_mb": seg.gpu_mem_mb(), "cached_images": seg.order})

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= MAX_BODY_BYTES:
                    return self._reply(413, {"ok": False, "error": f"请求体大小 {length} 不合法"})
                body = self.rfile.read(length)
                t0 = time.monotonic()
                if self.path == "/image":
                    image_id, width, height = seg.set_image(body)
                    reply = {"image_id": image_id, "width": width, "height": height}
                else:
                    req = json.loads(body)
                    threshold = float(req.get("threshold", seg.default_threshold))
                    if self.path == "/points":
                        reply = {"instances": seg.points(req["image_id"], req.get("points"), req.get("box"))}
                    elif self.path == "/text":
                        reply = {"results": seg.text(req["image_id"], req["prompts"], threshold)}
                    elif self.path == "/exemplar":
                        reply = {"instances": seg.exemplar(req["image_id"], req["box"], threshold)}
                    elif self.path == "/everything":
                        found, tried = seg.everything(req["image_id"], int(req.get("grid", 16)),
                                                      float(req.get("min_score", 0.85)),
                                                      float(req.get("max_area_frac", 0.3)))
                        reply = {"instances": found, "points_tried": tried}
                    else:
                        return self._reply(404, {"ok": False, "error": f"没有 {self.path}"})
                reply["ok"] = True
                reply["ms"] = round((time.monotonic() - t0) * 1000, 1)
                self._reply(200, reply)
            except Exception as e:                   # 坏请求、显存不够……回错误,服务不退出
                self._reply(500, {"ok": False, "error": f"{type(e).__name__}: {e}"})

        def log_message(self, fmt, *args):
            print(f"[sam3-click] {fmt % args}", flush=True)

    return Handler


def port_in_use(port):
    s = socket.socket()
    s.settimeout(0.3)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


def main():
    parser = argparse.ArgumentParser(description="SAM 3 交互演示服务(点 / 框 / 文本 / 示例框 / 全分割)")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--no-bf16", action="store_true")
    args = parser.parse_args()
    if port_in_use(8765):
        print("[sam3-click] 注意:常规 SAM 3 服务(8765)还开着,两个一起可能显存不够;不够就先把它停掉", flush=True)

    print("[sam3-click] 加载模型(带交互头)……", flush=True)
    seg = Interactive(args.threshold, bf16=not args.no_bf16)
    print(f"[sam3-click] 加载完成 {seg.load_s:.1f}s,显存 {seg.gpu_mem_mb()} MB", flush=True)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(seg))
    print(f"[sam3-click] http://127.0.0.1:{args.port}  (Ctrl+C 退出)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
