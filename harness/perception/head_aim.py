"""head_aim.py —— 算头部关节角,让头部相机的光轴对准 base_link 里的一个点。纯计算(PyKDL + numpy),不碰 ROS。

做法:URDF 建 base_link → head_link3 的链(5 个腰关节 + 3 个头关节),再乘上相机在 head_link3 里的外参,
得到相机位姿。只改头的 yaw(idx11,绕 z)和 pitch(idx13,绕 y),腰和歪头关节保持当前角度,
用 2×2 的高斯-牛顿迭代把“目标在相机里的水平角、竖直角”压到 0。每步都夹在 URDF 限位里。

相机光学系:x 右、y 下、z 前。目标在画面中央 ⇔ 它在相机系里 x = y = 0。
"""
import math

import numpy as np

from ..motion.kin import ChainKin
from .calib import CALIB
from .scene3d import pose_mat

YAW = "idx11_head_joint1"       # 绕 z 左右转头,正 = 向左
ROLL = "idx12_head_joint2"      # 绕 x 歪头,不动它
PITCH = "idx13_head_joint3"     # 绕 y 点头,正 = 低头
HEAD_JOINTS = (YAW, ROLL, PITCH)


class HeadAim:
    def __init__(self, urdf_str, calib=CALIB):
        self.kin = ChainKin(urdf_str, "base_link", "head_link3")
        # 和 scene3d.Rgbd 用同一个外参:深度和彩色逐像素对齐,3D 点就是在这个系里算出来的
        self.T_head_cam = pose_mat(np.array(calib["color_in_head_link3_xyz_qxyzw"], float))
        self.fx = calib["depth"]["fx"]
        self.fy = calib["depth"]["fy"]

    def camera_pose(self, joints):
        """joints:{关节名: 弧度}(至少含腰和头的 8 个)→ 相机在 base_link 里的 4x4。"""
        q = [joints[name] for name in self.kin.joint_names]
        p, R = self.kin.fk(q)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        return T @ self.T_head_cam

    def angles_off_center(self, joints, xyz):
        """目标偏离光轴的 (水平角, 竖直角),弧度。水平正 = 目标在画面右边,竖直正 = 在下边。"""
        T = self.camera_pose(joints)
        x, y, z = T[:3, :3].T @ (np.asarray(xyz, float) - T[:3, 3])
        return np.array([math.atan2(x, z), math.atan2(y, z)])

    def pixels_off_center(self, joints, xyz):
        """同上,换算成离画面中心(主点)多少像素。"""
        h, v = self.angles_off_center(joints, xyz)
        return [round(self.fx * math.tan(h), 1), round(self.fy * math.tan(v), 1)]

    def solve(self, joints, xyz, limits, margin_rad=0.03, iterations=30):
        """返回 (头部目标 {关节名: 弧度}, 剩余偏差 [水平°, 竖直°], 是否碰到限位)。

        limits:{关节名: (下限, 上限)},弧度。目标离限位至少留 margin_rad(gateway 自己还会再查一次)。
        """
        q = dict(joints)
        bounds = {}
        for name in (YAW, PITCH):
            low, high = limits.get(name, (-math.pi, math.pi))
            bounds[name] = (low + margin_rad, high - margin_rad)

        at_limit = False
        eps = 1e-4
        for _ in range(iterations):
            error = self.angles_off_center(q, xyz)
            if np.abs(error).max() < 1e-4:
                break
            # 数值雅可比:yaw、pitch 各动一点,看偏差怎么变
            J = np.zeros((2, 2))
            for col, name in enumerate((YAW, PITCH)):
                moved = dict(q)
                moved[name] += eps
                J[:, col] = (self.angles_off_center(moved, xyz) - error) / eps
            step = np.linalg.lstsq(J, -error, rcond=None)[0]
            step = np.clip(step, -0.3, 0.3)          # 一步最多 17°,防止离得远时跳过头
            at_limit = False
            for col, name in enumerate((YAW, PITCH)):
                low, high = bounds[name]
                value = q[name] + step[col]
                if value < low or value > high:
                    at_limit = True
                q[name] = min(max(value, low), high)

        residual = self.angles_off_center(q, xyz)
        target = {name: float(q[name]) for name in HEAD_JOINTS}
        return target, [round(math.degrees(v), 2) for v in residual], at_limit
