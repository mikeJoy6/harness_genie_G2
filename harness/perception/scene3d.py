"""scene3d.py —— 头部 RGBD 的三维换算（纯计算，无 ROS）：深度 → 3D 点 → base_link，空间范围筛选 + 连续性分割。

GDK 发的 /gdk/camera/head_depth 已经和 /gdk/camera/head_color 逐像素对齐（不用再配准）：2026-11-03 真机一帧，
深度跳变边到彩色 Canny 边的中位距离 —— 原始叠加 0.95px；按标定外参"配准"反而 3.28px（外参取反 3.82、只换内参 1.91）。
所以同一个像素 (u,v) 的彩色和深度就是同一个点。3D 用深度内参反投影，换算链同 genie_robot_agibot_g2_perception 的
head_depth_cloud（深度光学系 → 彩色光学系 → head_link3，用这条链拟合地面倾角 0.7°，已验证）。
标定见 record_dataset.CALIB（XToY = X 在 Y 里的位姿）。
"""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .calib import CALIB


def pose_mat(xyzq):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(xyzq[3:]).as_matrix()
    T[:3, 3] = xyzq[:3]
    return T


class Rgbd:
    def __init__(self, calib=CALIB):
        w, h = calib["image_size"]
        d = calib["depth"]
        fx, fy, cx, cy = d["fx"], d["fy"], d["cx"], d["cy"]
        v, u = np.mgrid[0:h, 0:w].astype(np.float32)
        self.ray = np.dstack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)])     # 深度光学系视线（无畸变）
        # head_link3 → 深度光学系 = GDK kHeadRGBDToHeadLink3 本身（"RGBD" = 深度相机/模组参考系），不再乘 depth_in_color。
        #   2026-11-04：原来按"RGBD=彩色、深度→彩色→head_link3"多乘了 depth_in_color（光学 x −2.38cm），
        #   纸板被算到真实位置左边（base +y）~3cm，吸盘实测稳定偏左 ~3cm。11-01 用地面拟合定的约定，
        #   地面拟合对横向平移不敏感，没区分出来。改回去：把 depth_in_color 乘回来即可。
        self.T_head_depth = pose_mat(np.array(calib["color_in_head_link3_xyz_qxyzw"], float))

    def points_base(self, depth_m, T_base_head):
        """深度图(米) → base_link 里的 3D 点 (H,W,3) 和相机原点（base_link）。无深度的像素点为 0。
        T_base_head：base_link → head_link3 的 4x4（来自 /gdk/tf）。"""
        T = T_base_head @ self.T_head_depth
        P = self.ray * depth_m[..., None].astype(np.float32)
        Pb = P @ T[:3, :3].T.astype(np.float32) + T[:3, 3].astype(np.float32)
        Pb[depth_m <= 0] = 0
        return Pb, T[:3, 3]


def depth_edges(z, abs_m=0.03, rel=0.02):
    """相邻像素深度差 > abs_m + rel·z 的地方（物体边缘、叠放纸板之间）标成边。"""
    z = z.astype(np.float32)
    e = np.zeros(z.shape, bool)
    for dy, dx in ((0, 1), (1, 0)):
        a = z[:z.shape[0] - dy, :z.shape[1] - dx]
        b = z[dy:, dx:]
        jump = (a > 0) & (b > 0) & (np.abs(a - b) > abs_m + rel * np.minimum(a, b))
        e[:z.shape[0] - dy, :z.shape[1] - dx] |= jump
        e[dy:, dx:] |= jump
    return e


def select_region(depth_m, Pb, cam, max_range, z_lo, z_hi, min_area, edge_abs=0.03, edge_rel=0.02):
    """空间范围筛选 + 连续性：
    离相机 ≤ max_range、base_link z ∈ [z_lo, z_hi] 的像素 → 去掉深度跳变边（把挨着但不连着的东西切开）
    → 8 邻域连通块，像素数 ≥ min_area 的留下。
    返回 (mask uint8 0/1, [(像素数, 质心(x,y)), ...] 按像素数从大到小)。"""
    ok = depth_m > 0
    rng = np.linalg.norm(Pb - cam.astype(np.float32), axis=2)
    zb = Pb[..., 2]
    m = ok & (rng <= max_range) & (zb >= z_lo) & (zb <= z_hi) & ~depth_edges(depth_m, edge_abs, edge_rel)
    n, lab, st, cen = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
    keep = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= min_area]
    mask = np.isin(lab, keep).astype(np.uint8)
    regs = sorted(((int(st[i, cv2.CC_STAT_AREA]), tuple(cen[i])) for i in keep), key=lambda r: -r[0])
    return mask, regs
