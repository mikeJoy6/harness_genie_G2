"""正运动学(FK):已知关节角,算 link 在哪。纯 numpy,不需要 ROS 在跑。

TF 里的坐标就是 robot_state_publisher 用这套方法算出来的。这里自己算一遍,和 TF 对比。

单独运行(不需要机器人):
    python3 learning/stage2_ros/kinematics.py
"""
import numpy as np
import urdf_parser_py.xml_reflection.core as urdf_core
from urdf_parser_py.urdf import URDF

# G2 的 URDF 里有一些 urdf_parser_py 不认识的标签,它会刷屏警告。这些不影响运动学,关掉
urdf_core.on_error = lambda *args, **kwargs: None


def load_urdf(xml):
    return URDF.from_xml_string(xml)


def chain_joints(robot, base, tip):
    """base → tip 路上经过的所有 joint(包括 fixed),按顺序。"""
    names = robot.get_chain(base, tip, joints=True, links=False)
    return [robot.joint_map[n] for n in names]


def rpy_to_matrix(rpy):
    """URDF 的 rpy:先绕 x 转 roll,再绕 y 转 pitch,再绕 z 转 yaw(都是固定轴)。"""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle_matrix(axis, angle):
    """绕任意单位轴 axis 转 angle 弧度(罗德里格斯公式)。"""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def make_transform(R=None, xyz=None):
    """拼一个 4×4 齐次矩阵:左上 3×3 是旋转,右上 3×1 是平移。"""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if xyz is not None:
        T[:3, 3] = xyz
    return T


def joint_transform(joint, q):
    """一个 joint 带来的变换 = 安装位置(origin,URDF 里写死的) × 关节运动(随 q 变)。"""
    origin = joint.origin
    T = make_transform(rpy_to_matrix(origin.rpy if origin else [0, 0, 0]),
                       origin.xyz if origin else [0, 0, 0])
    axis = joint.axis or [1, 0, 0]
    if joint.type in ("revolute", "continuous"):
        T = T @ make_transform(R=axis_angle_matrix(axis, q))
    elif joint.type == "prismatic":
        T = T @ make_transform(xyz=np.asarray(axis) * q)
    return T                                    # fixed:只有安装位置


def fk(robot, base, tip, joint_positions):
    """沿着 base → tip 的链,把每个 joint 的变换依次乘起来。返回 tip 在 base 下的 4×4 矩阵。"""
    T = np.eye(4)
    for joint in chain_joints(robot, base, tip):
        if joint.type == "fixed":
            q = 0.0
        elif joint.name in joint_positions:
            q = joint_positions[joint.name]
        else:
            raise KeyError(f"缺少关节角:{joint.name}")
        T = T @ joint_transform(joint, q)
    return T


def quat_to_matrix(q):
    """四元数 [x, y, z, w] → 3×3 旋转矩阵(TF 给的是四元数,对比时要转一下)。"""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_diff_deg(R1, R2):
    """两个旋转差了多少度。"""
    cos = (np.trace(R1.T @ R2) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


if __name__ == "__main__":
    # 离线自测:读 URDF 文件,所有关节都是 0 时,左臂末端在哪
    import config

    robot = load_urdf(config.URDF_FILE.read_text(encoding="utf-8"))
    print(f"URDF:{len(robot.links)} 个 link,{len(robot.joints)} 个 joint,根是 {robot.get_root()}")
    print(f"\n{config.BASE_FRAME} → {config.LEFT_END} 经过的 joint:")
    for j in chain_joints(robot, config.BASE_FRAME, config.LEFT_END):
        print(f"  {j.name:<24} {j.type:<9} 轴 {j.axis}")
    zeros = {j.name: 0.0 for j in robot.joints}
    T = fk(robot, config.BASE_FRAME, config.LEFT_END, zeros)
    print(f"\n全部关节为 0 时,左臂末端 xyz = {np.round(T[:3, 3], 4).tolist()} 米")
