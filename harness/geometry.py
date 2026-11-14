"""姿态换算。纯 numpy,不需要 ROS。

RPY 用 URDF/ROS 的约定:固定轴,先绕 x 转 roll,再绕 y 转 pitch,再绕 z 转 yaw;
和 harness/motion/algorithms.py 的 rpy_to_mat / mat_to_rpy 一致,只是这里的角度单位是度。
"""
import numpy as np


def rpy_deg_to_matrix(rpy_deg):
    r, p, y = np.radians(rpy_deg)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy_deg(R):
    pitch = float(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
    if abs(np.cos(pitch)) < 1e-6:                       # 万向锁:yaw 并进 roll
        roll, yaw = float(np.arctan2(-R[1, 2], R[1, 1])), 0.0
    else:
        roll, yaw = float(np.arctan2(R[2, 1], R[2, 2])), float(np.arctan2(R[1, 0], R[0, 0]))
    return [round(float(np.degrees(v)), 2) for v in (roll, pitch, yaw)]


def rotate_rpy_deg(rpy_deg, rotate_deg):
    """在 rpy_deg 这个姿态上,再绕 base_link 的 x、y、z 轴依次转 rotate_deg。返回新的 rpy(度)。"""
    R = rpy_deg_to_matrix(rotate_deg) @ rpy_deg_to_matrix(rpy_deg)
    return matrix_to_rpy_deg(R)


def rotation_between_deg(rpy_a, rpy_b):
    """两个姿态之间差了多少度(绕某根轴的最小转角)。"""
    Ra, Rb = rpy_deg_to_matrix(rpy_a), rpy_deg_to_matrix(rpy_b)
    cos = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
