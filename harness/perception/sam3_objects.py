"""sam3_objects.py —— SAM 3 的掩码 + 深度 → base_link 里的物体;以及把这次分割存进日志文件夹。纯计算,不碰 ROS。

    objects, rejected = objects_from_masks(results, depth_m, points_base, cam_xyz, principal_xy, min_depth_pixels)
    folder = save_segmentation(traces/sam3, "phone", bgr, depth_mm, results, objects, meta)

日志文件夹(每次分割一个):traces/sam3/<时间>_<短语>[_after]/
    color.jpg     原始彩色画面
    overlay.jpg   画面 + 掩码 + 编号 + 中心十字
    depth.png     深度图,uint16 毫米(离线可以重算 3D)
    mask_<编号>.png   每个物体的掩码,0/255
    result.json   短语、每个物体的编号/分数/坐标/距离/像素、头部位姿、耗时
"""
import json
import re
import time
from pathlib import Path

import cv2
import numpy as np

from .sam3_client import draw


def slug(text):
    """'cell phone' → 'cell_phone',用在编号和文件夹名里。"""
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_") or "object"


def objects_from_masks(results, depth_m, points_base, cam_xyz, principal_xy, min_depth_pixels=15):
    """每个掩码 → 一个物体。按离相机由近到远排序,编号 <短语>_1、<短语>_2……

    xyz_m:掩码里有效深度像素的 base_link 中位点(只取深度接近中位数的那层表面,去掉边缘飞点)。
    返回 (物体列表, 被丢掉的原因列表)。物体里带 "mask"(bool 数组),给模型之前要去掉。
    """
    found, rejected = [], []
    for r in results:
        for inst in r["instances"]:
            mask = inst["mask"]
            region = mask & (depth_m > 0)
            if int(region.sum()) < min_depth_pixels:
                rejected.append(f'{r["prompt"]} 分数 {inst["score"]:.2f}:掩码里有效深度只有 {int(region.sum())} 个像素')
                continue
            depth = float(np.median(depth_m[region]))
            surface = region & (np.abs(depth_m - depth) < 0.03)
            xyz = np.median(points_base[surface], axis=0)
            ys, xs = np.nonzero(mask)
            pixel = [float(xs.mean()), float(ys.mean())]
            found.append({"label": r["prompt"], "score": inst["score"],
                          "xyz_m": [round(float(v), 4) for v in xyz],
                          "distance_m": round(float(np.linalg.norm(xyz - np.asarray(cam_xyz))), 3),
                          "pixel_xy": [round(v, 1) for v in pixel],
                          "offset_from_center_px": [round(pixel[0] - principal_xy[0], 1),
                                                    round(pixel[1] - principal_xy[1], 1)],
                          "box_xyxy": inst["box_xyxy"], "area_px": inst["area_px"], "mask": mask})
    found.sort(key=_distance)
    for k, obj in enumerate(found, start=1):
        obj["id"] = f"{slug(obj['label'])}_{k}"
    return found, rejected


def _distance(obj):
    return obj["distance_m"]


def public(obj):
    """给模型看的字段:去掉掩码和框。"""
    return {key: obj[key] for key in ("id", "label", "score", "xyz_m", "distance_m", "offset_from_center_px")}


def save_segmentation(root, query, bgr, depth_mm, results, objects, meta, suffix=""):
    """把这次分割完整存下来,返回文件夹路径。"""
    folder = Path(root) / f"{time.strftime('%Y%m%d_%H%M%S')}_{slug(query)}{suffix}"
    n = 2
    while folder.exists():                       # 同一秒内分割两次
        folder = Path(f"{folder}_{n}")
        n += 1
    folder.mkdir(parents=True)

    cv2.imwrite(str(folder / "color.jpg"), bgr)
    cv2.imwrite(str(folder / "depth.png"), depth_mm.astype(np.uint16))
    cv2.imwrite(str(folder / "overlay.jpg"), overlay(bgr, results, objects))
    for obj in objects:
        cv2.imwrite(str(folder / f"mask_{obj['id']}.png"), obj["mask"].astype(np.uint8) * 255)

    record = {"query": query, "time": time.strftime("%Y-%m-%d %H:%M:%S"), **meta,
              "objects": [{**public(obj), "pixel_xy": obj["pixel_xy"], "box_xyxy": obj["box_xyxy"],
                           "area_px": obj["area_px"], "mask_file": f"mask_{obj['id']}.png"} for obj in objects]}
    (folder / "result.json").write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
    return folder


def overlay(bgr, results, objects):
    """draw() 的掩码 + 每个物体的编号 + 画面中心十字。"""
    image = draw(bgr, results)
    h, w = image.shape[:2]
    cv2.drawMarker(image, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)
    for obj in objects:
        x, y = int(obj["pixel_xy"][0]), int(obj["pixel_xy"][1])
        cv2.circle(image, (x, y), 3, (255, 255, 255), -1)
        cv2.putText(image, f'{obj["id"]} {obj["distance_m"]:.2f}m', (x + 5, y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, f'{obj["id"]} {obj["distance_m"]:.2f}m', (x + 5, y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return image
