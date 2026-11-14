"""用一张图测 SAM 3 服务(先启动 ~/sam3_service/server.py)。系统 Python 3.10 运行,不需要 ROS。

    python3 scripts/sam3_test_image.py recordings/head_20261112_015829/color_04.png "red block" "pen"
    python3 scripts/sam3_test_image.py 图片.jpg "red block" --threshold 0.3

结果图存到 traces/sam3_<图片名>.jpg(已 gitignore),终端打印每个实例的分数、框、面积和耗时。
"""
import argparse
import time
from pathlib import Path

import sys

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness.perception.sam3_client import Sam3Client, draw  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("prompts", nargs="+", help="英文短语,例如 \"green pepper\"")
    parser.add_argument("--threshold", type=float)
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    args = parser.parse_args()

    bgr = cv2.imread(args.image)
    if bgr is None:
        raise SystemExit(f"读不了图片 {args.image}")
    client = Sam3Client(args.url)
    print("服务状态:", client.health())

    t0 = time.monotonic()
    results = client.segment(bgr, args.prompts, args.threshold)
    total = (time.monotonic() - t0) * 1000
    for r in results:
        print(f'\n"{r["prompt"]}":{len(r["instances"])} 个')
        for inst in r["instances"]:
            print(f'  分数 {inst["score"]:.3f}  框 {inst["box_xyxy"]}  面积 {inst["area_px"]} 像素')
    print(f"\n耗时:总共 {total:.0f} ms(含传输);服务端 {client.last_timing_ms}")

    out_path = Path(__file__).resolve().parents[1] / "traces" / f"sam3_{Path(args.image).stem}.jpg"
    cv2.imwrite(str(out_path), draw(bgr, results))
    print(f"结果图:{out_path}")


if __name__ == "__main__":
    main()
