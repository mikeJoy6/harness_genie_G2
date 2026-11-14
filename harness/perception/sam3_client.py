"""sam3_client.py —— SAM 3 分割服务(~/sam3_service/server.py)的客户端。

服务跑在另一个 Python 3.12 环境里,这里只用标准库 + numpy + OpenCV,ROS 的 Python 3.10 能直接 import。

    from harness.perception.sam3_client import Sam3Client, draw
    client = Sam3Client()                                    # 默认 http://127.0.0.1:8765
    results = client.segment(bgr, ["red block", "pen"])      # 手上是解码好的图
    results = client.segment_jpeg(jpeg_bytes, ["pen"])       # 手上已经是 JPEG(比如 ROS CompressedImage),不再编解码
    for r in results:
        for inst in r["instances"]:
            inst["mask"]                                     # (H, W) bool
            inst["score"], inst["box_xyxy"], inst["area_px"]

完整说明:docs/sam3_service.md
"""
import base64
import json
import urllib.error
import urllib.request

import cv2
import numpy as np


class Sam3Client:
    def __init__(self, url="http://127.0.0.1:8765", timeout_s=30.0):
        self.url = url.rstrip("/")
        self.timeout_s = timeout_s
        self.last_timing_ms = None

    def _request(self, path, data=None, headers=None):
        request = urllib.request.Request(self.url + path, data=data, headers=headers or {})
        try:
            response = urllib.request.urlopen(request, timeout=self.timeout_s)
            try:
                return json.load(response)
            finally:
                response.close()
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"SAM 3 服务返回 {e.code}:{e.read().decode('utf-8', 'replace')[:300]}") from e
        except urllib.error.URLError as e:
            raise RuntimeError(f"连不上 SAM 3 服务 {self.url}(server.py 在跑吗?):{e.reason}") from e

    def health(self):
        return self._request("/health")

    def segment(self, bgr, prompts, threshold=None, jpeg_quality=95):
        """bgr:OpenCV 的 BGR 图。先编码成 JPEG,再交给 segment_jpeg。"""
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ok:
            raise ValueError("图片编码失败")
        return self.segment_jpeg(jpg, prompts, threshold)

    def segment_jpeg(self, jpeg, prompts, threshold=None):
        """jpeg:JPEG 字节,任何支持缓冲区协议的对象都行(bytes、numpy 数组、ROS 消息里的 array.array)。

        直接把这块内存作为 HTTP 请求体发出去,不做 base64,也不先复制一份。
        返回每个短语一组实例,掩码已解码成 bool 数组。
        """
        if isinstance(prompts, str):
            prompts = [prompts]
        meta = {"prompts": list(prompts)}
        if threshold is not None:
            meta["threshold"] = threshold
        headers = {"Content-Type": "image/jpeg",
                   "X-Sam3-Request": json.dumps(meta)}          # 只含 ASCII:json.dumps 默认会转义中文
        reply = self._request("/segment", memoryview(jpeg), headers)
        if not reply.get("ok"):
            raise RuntimeError(f"SAM 3 分割失败:{reply.get('error')}")
        for r in reply["results"]:
            for inst in r["instances"]:
                png = np.frombuffer(base64.b64decode(inst.pop("mask_png_b64")), dtype=np.uint8)
                inst["mask"] = cv2.imdecode(png, cv2.IMREAD_GRAYSCALE) > 127
        self.last_timing_ms = reply.get("timing_ms")
        return reply["results"]


PALETTE = [(0, 200, 255), (0, 255, 0), (255, 0, 255), (255, 128, 0), (0, 0, 255), (255, 255, 0)]


def draw(bgr, results, alpha=0.5):
    """把分割结果画到图上:同一个短语的实例用同一种颜色的半透明掩码 + 框 + "短语 分数"。返回新图,原图不动。"""
    out = bgr.copy()
    for k, r in enumerate(results):
        color = PALETTE[k % len(PALETTE)]
        for inst in r["instances"]:
            mask = inst["mask"]
            if mask.shape != out.shape[:2]:              # 掩码来自别的分辨率时跳过,不崩
                continue
            out[mask] = ((1 - alpha) * out[mask] + alpha * np.array(color)).astype(np.uint8)
            x0, y0, x1, y1 = [int(v) for v in inst["box_xyxy"]]
            cv2.rectangle(out, (x0, y0), (x1, y1), color, 2)
            cv2.putText(out, f'{r["prompt"]} {inst["score"]:.2f}', (x0, max(15, y0 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out
