"""touch_object 最后一段(不查 MoveIt 点云碰撞)的几何检查。纯 numpy,不需要 ROS。

2026-11-14 真机:“抓取最近的鼠标”时,朝下够不着,程序改用“肩 → 物体水平”接近,
最后 30cm 贴着桌面平推,不查点云 = 连桌面也不查(桌面只在点云里),张开的夹爪撞上桌面。
所以最后一段不再是“点云全不查”,而是:
  · 物体附近(object_radius_m 以内)的点云格放过 —— 物体本身就在点云里,要碰它;
  · 其余点云格照查:夹爪近似成一根胶囊(夹爪尖往回 length_m 的线段,外扩 radius_m),碰到就不走。
另外,放在台面上的物体不从侧面平推(rests_on_surface)。

所有坐标都在 base_link 系,单位米。
"""
import numpy as np


def outside_object(cloud, object_xyz, object_radius_m):
    """去掉物体附近 object_radius_m 以内的点云格,返回剩下的(最后一段不许碰的)。"""
    cloud = np.asarray(cloud, dtype=float).reshape(-1, 3)
    distance = np.linalg.norm(cloud - np.asarray(object_xyz, dtype=float), axis=1)
    return cloud[distance > object_radius_m]


def gripper_hits(tips, axes, cloud, radius_m, length_m):
    """夹爪在一串位姿上碰到几个点云格。

    tips:  (N,3) 夹爪尖位置
    axes:  (N,3) 工具轴单位向量(夹爪尖朝的方向)
    cloud: (M,3) 点云格中心
    夹爪 = 从 tip − length_m·axis 到 tip 的线段,外扩 radius_m。返回碰到的格数(去重)。
    """
    cloud = np.asarray(cloud, dtype=float).reshape(-1, 3)
    if len(cloud) == 0:
        return 0
    hit = np.zeros(len(cloud), dtype=bool)
    for tip, axis in zip(np.asarray(tips, dtype=float), np.asarray(axes, dtype=float)):
        back = tip - length_m * axis                    # 线段另一头(靠法兰那端)
        v = cloud - back
        t = np.clip(v @ axis / length_m, 0.0, 1.0)      # 每个格在线段上最近点的位置(0 = 法兰端,1 = 夹爪尖)
        nearest = back + np.outer(t * length_m, axis)
        hit |= np.linalg.norm(cloud - nearest, axis=1) < radius_m
    return int(hit.sum())


def rests_on_surface(cloud, object_xyz, ring_m=(0.06, 0.20), below_m=(0.005, 0.10), min_cells=5):
    """物体是不是放在台面上:物体周围一圈(水平距离在 ring_m 之间)、比物体低 below_m 的地方
    有至少 min_cells 个点云格,就当它下面是桌子 / 架子这种平面。

    物体坐标是可见表面的中位点(鼠标就是顶面),台面在它下面几厘米、四周都有。
    举在空中的手四周下方通常是空的(小臂可能被算进来,那样只是不从侧面去碰,不会撞)。"""
    cloud = np.asarray(cloud, dtype=float).reshape(-1, 3)
    if len(cloud) == 0:
        return False
    obj = np.asarray(object_xyz, dtype=float)
    horizontal = np.linalg.norm(cloud[:, :2] - obj[:2], axis=1)
    lower = obj[2] - cloud[:, 2]
    in_ring = (horizontal >= ring_m[0]) & (horizontal <= ring_m[1])
    in_band = (lower >= below_m[0]) & (lower <= below_m[1])
    return int((in_ring & in_band).sum()) >= min_cells
