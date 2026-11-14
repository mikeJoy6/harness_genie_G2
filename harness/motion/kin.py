#!/usr/bin/env python3
"""kin.py —— 运动学：从 URDF 建 KDL 链，算 FK、雅可比 J、阻尼伪逆 J⁺、KDL LMA 位姿 IK。
纯计算，不碰 ROS(URDF 字符串从外面传进来)，所以能离线单测。

移植自 sanofi_trash_grasping/src/tianyi_arm/tianyi_arm/kin.py（天工 → G2 只换了链两端的名字）：
  G2 手臂链：arm_base_link → arm_{l,r}_end_link，正好 7 个手臂关节（idx2x / idx6x）。
  arm_base_link 挂在腰末端(body_link5，固定关节 idx20_arm_base_joint)，相当于天工的 waist_yaw_link。
  腰链：base_link → arm_base_link，5 个腰关节 idx01~idx05，用来把 base_link 系的目标换到手臂根系。
"""
import numpy as np
import PyKDL as kdl
from urdf_parser_py import xml_reflection
from urdf_parser_py.urdf import URDF

# G2 URDF 里有 collision name / collision_checking 等 urdfdom 不认识的标签，每次解析都刷几十行
#   "Unknown attribute"。都是无关的扩展字段，静音掉(只影响 urdf_parser_py 的告警输出)。
xml_reflection.core.on_error = lambda *_args, **_kw: None

ARM_ROOT = "arm_base_link"
ARM_TIP = {"left": "arm_l_end_link", "right": "arm_r_end_link"}
# robot_harness 加:URDF 里有 TCP(hardware.launch end_effector:=true 加载的 G2_t2_crsB_ee.urdf:end_link 沿 z 平移 0.128,
#   即夹爪 / 末端长方体的端面中心)就把手臂链的末端设到 TCP。这样所有目标点、末端位姿、高度限制都是“夹爪尖在哪”。
TCP_TIP = {"left": "tcp_l_link", "right": "tcp_r_link"}


def arm_tip(urdf_str, arm):
    """这份 URDF 里手臂链的末端:有 TCP 用 TCP,没有用法兰 arm_*_end_link。"""
    if f'link name="{TCP_TIP[arm]}"' in urdf_str:
        return TCP_TIP[arm]
    return ARM_TIP[arm]
BASE = "base_link"


# ── URDF 的一个 joint/pose → KDL 的对应物(这段就是 kdl_parser_py 干的活，ROS2 没带) ──
def _pose2frame(p):
    xyz = list(p.xyz) if (p and p.xyz) else [0, 0, 0]
    rpy = list(p.rpy) if (p and p.rpy) else [0, 0, 0]
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def _joint2kdl(j):
    if j.type == "fixed":
        return kdl.Joint(j.name, kdl.Joint.Fixed)
    o = _pose2frame(j.origin)
    axis = kdl.Vector(*(j.axis if j.axis else [0, 0, 1]))
    if j.type == "prismatic":
        return kdl.Joint(j.name, o.p, o.M * axis, kdl.Joint.TransAxis)
    return kdl.Joint(j.name, o.p, o.M * axis, kdl.Joint.RotAxis)   # revolute / continuous


def _build_tree(robot):
    # 从根 link 广度优先，把每个 joint 建成 KDL Segment 挂上去 → 整棵 KDL 树
    tree = kdl.Tree(robot.get_root())
    queue = [robot.get_root()]
    while queue:
        parent = queue.pop(0)
        for jname, child in robot.child_map.get(parent, []):
            j = robot.joint_map[jname]
            tree.addSegment(kdl.Segment(child, _joint2kdl(j), _pose2frame(j.origin)), parent)
            queue.append(child)
    return tree


class ChainKin:
    """给一段 URDF + 链的两端，提供 fk() / jacobian() / ik()。"""
    def __init__(self, urdf_str, root=ARM_ROOT, tip=ARM_TIP["left"]):
        robot = URDF.from_xml_string(urdf_str)
        self.root, self.tip = root, tip
        self.chain = _build_tree(robot).getChain(root, tip)
        self.n = self.chain.getNrOfJoints()
        self._jac = kdl.ChainJntToJacSolver(self.chain)
        self._fk = kdl.ChainFkSolverPos_recursive(self.chain)
        self._ik_lma = kdl.ChainIkSolverPos_LMA(self.chain)   # C++ 位姿 IK(不管限位，调用方自查)
        self.joint_names = [self.chain.getSegment(i).getJoint().getName()
                            for i in range(self.chain.getNrOfSegments())
                            if self.chain.getSegment(i).getJoint().getType() != kdl.Joint.Fixed]
        # 关节位置限位(URDF)，按 joint_names 顺序；无限位(continuous)记 ±inf
        lo, hi = [], []
        for jn in self.joint_names:
            lim = getattr(robot.joint_map.get(jn), "limit", None)
            if lim is not None and lim.lower is not None and robot.joint_map[jn].type != "continuous":
                lo.append(lim.lower); hi.append(lim.upper)
            else:
                lo.append(-np.inf); hi.append(np.inf)
        self.lower = np.array(lo)
        self.upper = np.array(hi)

    def _arr(self, q):
        a = kdl.JntArray(self.n)
        for i, v in enumerate(q):
            a[i] = float(v)
        return a

    def fk(self, q):
        """当前关节角 → 末端 (位置3, 旋转3x3)，都在 root 系。"""
        f = kdl.Frame()
        self._fk.JntToCart(self._arr(q), f)
        pos = np.array([f.p[i] for i in range(3)])
        rot = np.array([[f.M[r, c] for c in range(3)] for r in range(3)])
        return pos, rot

    def segment_positions(self, q):
        """链上每个 segment 末端的位置（root 系），(n_seg, 3)。离线碰撞测试/可视化用。"""
        arr = self._arr(q)
        out = []
        for i in range(1, self.chain.getNrOfSegments() + 1):
            f = kdl.Frame()
            self._fk.JntToCart(arr, f, i)
            out.append([f.p[0], f.p[1], f.p[2]])
        return np.array(out)

    def jacobian(self, q):
        """当前关节角 → 6×n 雅可比 J（root 系，参考点在末端原点）。"""
        J = kdl.Jacobian(self.n)
        self._jac.JntToJac(self._arr(q), J)
        return np.array([[J[r, c] for c in range(self.n)] for r in range(6)])

    def ik(self, pos, rot, q_init):
        """KDL LMA 解 6D 位姿(pos3 + 3×3 rot，root 系) → 关节角 或 None(没收敛)。
        ★不保证在限位内、也可能只是近似解★——调用方拿到后要自己 fk 复核 + 查限位。"""
        F = kdl.Frame(kdl.Rotation(*[float(v) for v in np.asarray(rot).reshape(-1)]),
                      kdl.Vector(*[float(v) for v in pos]))
        qout = kdl.JntArray(self.n)
        if self._ik_lma.CartToJnt(self._arr(q_init), F, qout) < 0:
            return None
        return np.array([qout[i] for i in range(self.n)])


def arm_kin(urdf_str, arm):
    """G2 手臂链：arm_base_link → arm_{l,r}_end_link（7 关节）。"""
    return ChainKin(urdf_str, ARM_ROOT, arm_tip(urdf_str, arm))


def body_kin(urdf_str):
    """G2 腰链：base_link → arm_base_link（5 个腰关节），只用来做 base_link ↔ 手臂根系的换算。"""
    return ChainKin(urdf_str, BASE, ARM_ROOT)


def dls_pinv(J, lam=0.05):
    """阻尼最小二乘伪逆 J⁺ = Jᵀ(JJᵀ+λ²I)⁻¹，n×6。λ 越大越稳越糊。"""
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(J.shape[0]))
