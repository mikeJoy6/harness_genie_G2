"""从一帧 RGBD 中按 HSV 颜色找物体。纯计算,不依赖 ROS,可以离线测试。"""
import cv2
import numpy as np


def color_mask(bgr, ranges):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in ranges:
        mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))   # 补上高光造成的小洞
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def find_objects(bgr, depth_m, points_base, fx, ranges, cfg):
    """返回 (候选列表, 被排除的原因列表)。每个候选的 xyz_m 是可见表面在 base_link 下的中位点。"""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(color_mask(bgr, ranges), connectivity=8)
    found, rejected = [], []
    for i in range(1, count):
        pixels = int(stats[i, cv2.CC_STAT_AREA])
        if pixels < cfg["min_pixels"]:
            continue
        region = (labels == i) & (depth_m > 0)
        if int(region.sum()) < cfg["min_valid_depth_pixels"]:
            rejected.append(f"{pixels} 像素的区域没有足够的有效深度")
            continue
        depth = float(np.median(depth_m[region]))
        size = max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]) * depth / fx
        if not cfg["min_size_m"] <= size <= cfg["max_size_m"]:
            rejected.append(f"物体宽约 {size * 100:.1f}cm,超出尺寸范围")
            continue
        surface = region & (np.abs(depth_m - depth) < 0.03)     # 去掉边缘飞点
        found.append({"xyz_m": np.median(points_base[surface], axis=0).tolist(), "pixels": pixels,
                      "depth_m": round(depth, 3), "size_m": round(float(size), 3),
                      "x": int(stats[i, cv2.CC_STAT_LEFT]), "y": int(stats[i, cv2.CC_STAT_TOP]),
                      "w": int(stats[i, cv2.CC_STAT_WIDTH]), "h": int(stats[i, cv2.CC_STAT_HEIGHT])})
    return found, rejected


def merge_frames(per_frame, min_good, max_spread, reference):
    """per_frame:每帧的候选列表。每帧选离 reference(左臂末端)最近的候选,再要求各帧位置稳定。
    返回 (候选, 错误)。"""
    reference = np.asarray(reference, dtype=float)

    def distance(candidate):
        return float(np.linalg.norm(np.asarray(candidate["xyz_m"]) - reference))

    chosen = [min(frame, key=distance) for frame in per_frame if frame]
    if len(chosen) < min_good:
        return None, f"{len(per_frame)} 帧中只有 {len(chosen)} 帧看到这种颜色的物体"
    points = np.array([c["xyz_m"] for c in chosen])
    center = np.median(points, axis=0)
    spread = float(np.linalg.norm(points - center, axis=1).max())
    if spread > max_spread:
        return None, f"目标位置不稳定,各帧偏差 {spread * 100:.1f}cm,可能有多个物体离手臂差不多远"
    return {**chosen[-1], "xyz_m": center.tolist(), "candidates": max(len(frame) for frame in per_frame),
            "distance_to_left_end_m": round(float(np.linalg.norm(center - reference)), 3)}, None
