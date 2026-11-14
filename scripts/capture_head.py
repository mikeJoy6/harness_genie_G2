"""录几帧 G2 头部相机:彩色 + 深度(毫米)+ base_link→head_link3 位姿。只订阅,机器人不动。

格式和 robot_harness 的 DatasetSource 一样(frames.jsonl),离线时可以直接拿来测“掩码 + 深度 → 3D 坐标”。

    source ~/agi@g2_robot/env_g2.sh
    python3 scripts/capture_head.py            # 默认 5 帧,存到 recordings/head_<时间>/(已 gitignore)
    python3 scripts/capture_head.py --frames 10
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness.perception.rgbd_source import RosSource  # noqa: E402


def matrix_to_xyz_qxyzw(T):
    R, t = T[:3, :3], T[:3, 3]
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return [float(v) for v in (*t, x, y, z, w)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--interval", type=float, default=0.5, help="两帧之间隔几秒")
    args = parser.parse_args()

    out = Path(__file__).resolve().parents[1] / "recordings" / time.strftime("head_%Y%m%d_%H%M%S")
    out.mkdir(parents=True)
    source = RosSource("sam3_capture_head")
    saved, deadline = [], time.monotonic() + 20 + args.frames * args.interval
    try:
        while len(saved) < args.frames and time.monotonic() < deadline:
            frame = source.next(None)
            if frame is None:
                time.sleep(0.02)
                continue
            bgr, depth_mm, T, _ = frame
            k = len(saved)
            cv2.imwrite(str(out / f"color_{k:02d}.png"), bgr)
            cv2.imwrite(str(out / f"depth_{k:02d}.png"), depth_mm.astype(np.uint16))
            saved.append({"color": f"color_{k:02d}.png", "depth": f"depth_{k:02d}.png",
                          "base_link_T_head_link3_xyz_qxyzw": matrix_to_xyz_qxyzw(T),
                          "time": time.strftime("%Y-%m-%d %H:%M:%S")})
            valid = float((depth_mm > 0).mean())
            print(f"第 {k + 1} 帧:彩色 {bgr.shape},深度 {depth_mm.shape} 有效 {valid:.0%}", flush=True)
            time.sleep(args.interval)
    finally:
        source.close()
    (out / "frames.jsonl").write_text("".join(json.dumps(f) + "\n" for f in saved))
    print(f"存了 {len(saved)} 帧 → {out}")
    if not saved:
        sys.exit(1)


if __name__ == "__main__":
    main()
