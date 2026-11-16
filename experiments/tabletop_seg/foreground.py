"""桌面全分割 第 1 步:找出“桌上有东西”的像素(前景掩码)。纯离线,不需要 ROS,不需要 SAM。

思路(docs 见本目录 README):
    桌面平面   多帧深度取中位数 → base_link 点 → RANSAC 拟合水平面 → 每个像素离桌面多高
    桌面区域   平面内点最大的那块,取凸包:只在桌面里面找东西(人、地面、桌外的东西不要)
    三种前景   深度:高出桌面 > height_mm          (白鼠标这种和桌面同色的靠它)
               颜色:和“附近的桌面”颜色不一样     (钥匙这种几毫米厚、深度看不出的靠它)
               空洞:桌面区域里深度读不到的小块   (透明杯、黑色 / 反光的东西)

    python3 experiments/tabletop_seg/foreground.py recordings/head_20261115_184218
    python3 experiments/tabletop_seg/foreground.py recordings/head_20261115_*      # 每个场景出一张图
图存到 experiments/tabletop_seg/out/(已 gitignore,画面里可能有人)。
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from harness.perception.scene3d import Rgbd, pose_mat  # noqa: E402

OUT = Path(__file__).resolve().parent / "out"


# ---------------- 读数据 ----------------
def load_scene(folder):
    """一个场景的所有帧:彩色列表、深度列表(毫米)、头部位姿(取第一帧)。"""
    folder = Path(folder)
    frames = []
    for line in (folder / "frames.jsonl").read_text().splitlines():
        if line.strip():
            frames.append(json.loads(line))
    colors = []
    depths = []
    for f in frames:
        colors.append(cv2.imread(str(folder / f["color"])))
        depths.append(cv2.imread(str(folder / f["depth"]), cv2.IMREAD_UNCHANGED))
    T = pose_mat(np.array(frames[0]["base_link_T_head_link3_xyz_qxyzw"], float))
    return colors, depths, T


def fuse_depth(depths):
    """多帧深度逐像素取中位数(只算有深度的帧);一半以上的帧没深度的像素当作没有深度。
    同时返回每个像素在各帧间的标准差(毫米),看噪声有多大。"""
    stack = np.stack(depths).astype(np.float32)          # (帧数, H, W)
    valid = stack > 0
    count = valid.sum(axis=0)
    stack[~valid] = np.nan
    fused = np.nanmedian(stack, axis=0)
    spread = np.nanstd(stack, axis=0)
    enough = count >= len(depths) / 2
    fused[~enough] = 0
    spread[~enough] = 0
    return np.nan_to_num(fused), np.nan_to_num(spread)


# ---------------- 桌面平面 ----------------
def fit_table_plane(points, valid, z_range=(0.4, 1.2), threshold_m=0.008, iterations=400, seed=0):
    """在 base_link z 落在 z_range 的点里,RANSAC 找最大的“接近水平”的平面。
    返回 (法向量 n,常数 d):平面是 n·p + d = 0,n 朝上;再用内点最小二乘精修一次。"""
    candidate = valid & (points[..., 2] > z_range[0]) & (points[..., 2] < z_range[1])
    P = points[candidate]
    if len(P) < 1000:
        raise RuntimeError("桌面高度范围内的点太少")
    rng = np.random.default_rng(seed)
    sample = P[rng.choice(len(P), size=min(len(P), 20000), replace=False)]
    best_count = 0
    best = None
    for _ in range(iterations):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        if n[2] < 0:
            n = -n
        if n[2] < 0.95:                                  # 只要接近水平的面(倾斜 < 18°)
            continue
        d = -float(n @ a)
        count = int((np.abs(sample @ n + d) < threshold_m).sum())
        if count > best_count:
            best_count = count
            best = (n, d)
    n, d = best
    inliers = P[np.abs(P @ n + d) < threshold_m]         # 最小二乘精修:拟合 z = ax + by + c
    A = np.column_stack([inliers[:, 0], inliers[:, 1], np.ones(len(inliers))])
    coef, _, _, _ = np.linalg.lstsq(A, inliers[:, 2], rcond=None)
    n = np.array([-coef[0], -coef[1], 1.0])
    scale = np.linalg.norm(n)
    return n / scale, -coef[2] / scale


def table_region(height, valid, inlier_m=0.008):
    """平面内点里面积最大的一块 → 凸包 → 桌面区域掩码。桌上的东西是内点块里的“洞”,凸包把它们包进来。"""
    inlier = valid & (np.abs(height) < inlier_m)
    inlier = cv2.morphologyEx(inlier.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(inlier, connectivity=8)
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(labels == biggest)
    hull = cv2.convexHull(np.column_stack([xs, ys]).astype(np.int32))
    region = np.zeros(height.shape, np.uint8)
    cv2.fillConvexPoly(region, hull, 1)
    region = cv2.erode(region, np.ones((7, 7), np.uint8))    # 桌边一圈不要(边缘深度不准)
    return region > 0, labels == biggest


# ---------------- 三种前景 ----------------
def depth_foreground(height, valid, region, min_mm, max_m=0.6):
    return region & valid & (height > min_mm / 1000.0) & (height < max_m)


def color_foreground(bgr, region, table_pixels, chroma_min=14.0, dark_ratio=0.62, bright_ratio=1.25):
    """和“附近的桌面”比颜色:
    色度(Lab 的 a、b)离桌面色度远 → 有颜色的东西;
    亮度比附近桌面暗很多 → 深色的东西(钥匙、线、黑充电器);阴影只是稍暗、色度不变,用 dark_ratio 挡掉大部分;
    附近桌面的亮度:只用桌面像素,大核模糊后按权重归一(东西所在的地方用周围桌面补)。"""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    L, a, b = lab[..., 0], lab[..., 1], lab[..., 2]
    weight = table_pixels.astype(np.float32)
    kernel = (61, 61)
    L_bg = cv2.blur(L * weight, kernel) / np.maximum(cv2.blur(weight, kernel), 1e-3)
    a_ref = float(np.median(a[table_pixels]))
    b_ref = float(np.median(b[table_pixels]))
    chroma = np.hypot(a - a_ref, b - b_ref)
    ratio = L / np.maximum(L_bg, 1.0)
    colored = chroma > chroma_min
    dark = ratio < dark_ratio
    bright = ratio > bright_ratio
    return region & (colored | dark | bright), chroma, ratio


def edge_foreground(bgr, region, ratio, chroma, height, border_px=12, fill_max_px=2500):
    """白桌面没有纹理,图里的边几乎都是东西的轮廓:灰色金属(齿轮、勺子、剪刀刃)颜色和深度都不明显,但边很清楚。
    Canny 找边 → 加粗、闭合 → 封闭的小轮廓逐个检查,里面“不像桌面”才填实:
        平均亮度比附近桌面暗(ratio < 0.9)、或平均色度 > 8、或平均高出桌面 > 3mm。
    线缆和物体轮廓围出来的一片空桌面,里面又白又平,不填(2026-11-15 第一版全填,把空桌面也填进来了)。
    桌子边沿 border_px 以内不算(桌边本身就是一条边)。"""
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    edges = cv2.Canny(gray, 40, 100) > 0
    inner = cv2.erode(region.astype(np.uint8), np.ones((border_px, border_px), np.uint8)) > 0
    edges = edges & inner
    thick = cv2.dilate(edges.astype(np.uint8), np.ones((3, 3), np.uint8))
    closed = cv2.morphologyEx(thick, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    filled = np.zeros(region.shape, np.uint8)
    contours, _ = cv2.findContours(closed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > fill_max_px:
            continue
        blob = np.zeros(region.shape, np.uint8)
        cv2.drawContours(blob, [c], -1, 1, -1)
        inside = blob > 0
        if not inside.any():
            continue
        darker = float(np.mean(ratio[inside])) < 0.9
        colored = float(np.mean(chroma[inside])) > 8.0
        raised = float(np.mean(height[inside])) > 0.003
        if darker or colored or raised:
            filled |= blob
    return (filled > 0) & inner


def hole_foreground(valid, region, max_area_px=4000):
    """桌面区域里没有深度的小块(大块多半是桌边 / 远处,不算)。"""
    holes = (region & ~valid).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(holes, connectivity=8)
    keep = np.zeros(valid.shape, bool)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if 8 <= area <= max_area_px:
            keep |= labels == i
    return keep


def clean(mask, min_area=12):
    """去掉零星噪点(小于 min_area 像素的块)。"""
    m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    keep = np.zeros(mask.shape, bool)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            keep |= labels == i
    return keep


# ---------------- 一个场景 ----------------
def run_scene(folder, args, rgbd):
    colors, depths, T = load_scene(folder)
    depth_mm, spread_mm = fuse_depth(depths)
    valid = depth_mm > 0
    points, _ = rgbd.points_base(depth_mm / 1000.0, T)
    n, d = fit_table_plane(points, valid)
    height = points @ n + d
    height[~valid] = 0
    region, table_pixels = table_region(height, valid)
    table_pixels = table_pixels & region

    residual_mm = np.abs(height[table_pixels]) * 1000
    print(f"{Path(folder).name}:桌面高 {-d / n[2]:.3f}m,倾斜 {np.degrees(np.arccos(n[2])):.1f}°;"
          f"桌面点离平面 中位 {np.median(residual_mm):.1f}mm、95% {np.percentile(residual_mm, 95):.1f}mm;"
          f"帧间抖动 中位 {np.median(spread_mm[table_pixels]):.1f}mm")

    color = colors[0]
    fg_depth = clean(depth_foreground(height, valid, region, args.height_mm))
    fg_color, chroma, ratio = color_foreground(color, region, table_pixels & ~fg_depth)
    fg_color = clean(fg_color)
    fg_hole = hole_foreground(valid, region)
    fg_edge = clean(edge_foreground(color, region, ratio, chroma, height))
    inner = cv2.erode(region.astype(np.uint8), np.ones((12, 12), np.uint8)) > 0
    fg_color = fg_color & inner                           # 桌子边沿一圈的颜色变化不算(桌边)
    fg = clean(fg_depth | fg_color | fg_hole | fg_edge)

    # 图:左上原图 + 桌面区域;右上三种前景(蓝深度、红颜色、黄空洞,重叠时混色);
    #     左下高度图;右下合起来的前景
    view = color.copy()
    contours, _ = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(view, contours, -1, (0, 255, 0), 2)
    layers = color.copy() // 3
    layers[fg_depth] = layers[fg_depth] + np.array([200, 60, 0], np.uint8) // 1
    layers[fg_color] = np.clip(layers[fg_color].astype(int) + [0, 40, 200], 0, 255).astype(np.uint8)
    layers[fg_hole] = (0, 230, 255)
    layers[fg_edge & ~fg_depth & ~fg_color & ~fg_hole] = (0, 255, 0)              # 只有边缘线索抓到的:绿
    hmap = np.clip(height * 1000 / 60.0 * 255, 0, 255).astype(np.uint8)          # 0~60mm → 0~255
    hmap = cv2.applyColorMap(hmap, cv2.COLORMAP_TURBO)
    hmap[~region | ~valid] = 0
    merged = color.copy()
    merged[~fg] = (merged[~fg] * 0.25).astype(np.uint8)
    top = np.hstack([label(view, "table region"), label(layers, "blue=depth red=color yellow=hole green=edge-only")])
    bottom = np.hstack([label(hmap, "height 0-60mm"), label(merged, "foreground (union)")])
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{Path(folder).name}_fg.jpg"
    cv2.imwrite(str(path), np.vstack([top, bottom]))
    print(f"    前景像素:深度 {fg_depth.sum()}、颜色 {fg_color.sum()}、空洞 {fg_hole.sum()}、边缘 {fg_edge.sum()}"
          f" → 合 {fg.sum()};图 {path}")
    return {"depth": fg_depth, "color": fg_color, "hole": fg_hole, "edge": fg_edge, "all": fg, "height": height,
            "region": region, "valid": valid, "points": points, "color_image": color}


def label(img, text):
    img = img.copy()
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def main():
    parser = argparse.ArgumentParser(description="桌面前景掩码(深度 + 颜色 + 空洞)")
    parser.add_argument("scenes", nargs="+", help="recordings/head_* 文件夹")
    parser.add_argument("--height-mm", type=float, default=6.0, help="高出桌面多少毫米算东西")
    args = parser.parse_args()
    rgbd = Rgbd()
    for folder in args.scenes:
        run_scene(folder, args, rgbd)


if __name__ == "__main__":
    main()
