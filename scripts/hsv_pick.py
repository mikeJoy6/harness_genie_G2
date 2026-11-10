"""点选物体颜色,保存到 configs/colors.yaml,之后模型就能按这个名字定位。

  source ~/agi@g2_robot/env_g2.sh
  python3 scripts/hsv_pick.py --name 青椒                      实时头部画面
  python3 scripts/hsv_pick.py --name 青椒 --image 图片.jpg      用一张图片
  python3 scripts/hsv_pick.py --name 青椒 --image 图片.jpg --pixel 320,200      不开窗口

窗口操作:左键点物体(多点几处,包括亮面和暗面),紫色轮廓是当前识别结果;
c 清空,s 保存并退出,q 不保存退出。
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness.backends.color_object import color_mask
from harness.colors import range_from_samples, save_color

COLORS_PATH = ROOT / "configs/colors.yaml"
WINDOW = "hsv_pick"


def sample_at(bgr, x, y, radius=2):
    h, w = bgr.shape[:2]
    patch = bgr[max(y - radius, 0):min(y + radius + 1, h), max(x - radius, 0):min(x + radius + 1, w)]
    return cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)


def save(name, samples):
    ranges = range_from_samples(np.concatenate(samples))
    save_color(COLORS_PATH, name, ranges)
    print(f"已保存 {name}: {ranges}", flush=True)


def interactive(name, next_frame):
    state = {"bgr": None, "samples": []}

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and state["bgr"] is not None:
            state["samples"].append(sample_at(state["bgr"], x, y))
            print(f"采样 ({x}, {y}):HSV 中位数 {np.median(state['samples'][-1], axis=0).astype(int).tolist()}", flush=True)

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, on_mouse)
    while True:
        frame = next_frame()
        if frame is not None:
            state["bgr"] = frame
        if state["bgr"] is not None:
            view = state["bgr"].copy()
            if state["samples"]:
                mask = color_mask(state["bgr"], range_from_samples(np.concatenate(state["samples"])))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(view, contours, -1, (255, 0, 255), 2)
            cv2.imshow(WINDOW, view)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("c"):
            state["samples"].clear()
        elif key == ord("q"):
            return
        elif key == ord("s"):
            if state["samples"]:
                save(name, state["samples"])
                return
            print("还没有采样,先用左键点物体", flush=True)


def main():
    parser = argparse.ArgumentParser(description="点选物体颜色并保存")
    parser.add_argument("--name", required=True, help="颜色或物体的名字,如 青椒")
    parser.add_argument("--image", help="用图片代替实时画面")
    parser.add_argument("--pixel", help="不开窗口,直接采样这个像素,格式 x,y;需要 --image")
    args = parser.parse_args()

    if args.pixel:
        if not args.image:
            parser.error("--pixel 需要配合 --image")
        bgr = cv2.imread(args.image)
        if bgr is None:
            parser.error(f"读不到图片 {args.image}")
        x, y = (int(v) for v in args.pixel.split(","))
        save(args.name, [sample_at(bgr, x, y)])
        return

    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            parser.error(f"读不到图片 {args.image}")
        interactive(args.name, lambda: image)
        return

    from task_board_pose.vision.rgbd_source import RosSource    # 放这里,用图片时不需要 ROS
    source = RosSource("robot_harness_hsv_pick")

    def next_frame():
        frame = source.next(None)
        return None if frame is None else frame[0]

    try:
        interactive(args.name, next_frame)
    finally:
        source.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
