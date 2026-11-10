"""Local pose IK helpers, copied from g1_pointcloud_grasping genie_motion/kinematics.py.

ChainKin itself is the G2 one (../kin.py), so ik_lab, the keyboard jog and arm_motion share one
URDF -> KDL model. Everything below ChainKin is unchanged from genie_motion.
"""

from dataclasses import dataclass
import math

import numpy as np
import PyKDL as kdl

from ..kin import ChainKin  # noqa: F401  (re-exported for ik_lab modules)


def rotation_quaternion(rotation):
    """Quaternion in XYZW order, from an actual rotation matrix."""
    return np.array(kdl.Rotation(*[float(v) for v in rotation.ravel()]).GetQuaternion())


def rotation_vector(current, target):
    """SO(3) log of target @ current.T, expressed in the chain root frame."""
    error = target @ current.T
    vector = kdl.Rotation(*[float(v) for v in error.ravel()]).GetRot()
    return np.array([vector[i] for i in range(3)])


def rotation_about(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == 0:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 1:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


@dataclass
class IkResult:
    q: np.ndarray
    converged: bool
    position_error: float
    rotation_error: float
    iterations: int
    reason: str = ''
    details: str = ''


def solve_pose(kin, q_start, position, rotation, damping=0.05, max_iters=100):
    """Iterate on FK residual; no random restart and no endpoint IK pre-search."""
    q = np.array(q_start, dtype=float, copy=True)
    for iteration in range(max_iters + 1):
        p, rot = kin.fk(q)
        ep, er = position - p, rotation_vector(rot, rotation)
        pos_error, rot_error = float(np.linalg.norm(ep)), float(np.linalg.norm(er))
        in_limits = np.all(q >= kin.lower) and np.all(q <= kin.upper)
        if in_limits and pos_error < 1e-4 and rot_error < 1e-3:
            return IkResult(q, True, pos_error, rot_error, iteration)
        if iteration == max_iters:
            break
        step = np.concatenate((np.clip(ep, -0.002, 0.002), np.clip(er, -0.01, 0.01)))
        jac = kin.jacobian(q)
        # J^T (J J^T + lambda^2 I)^-1 e; solve avoids forming an explicit inverse.
        dq = jac.T @ np.linalg.solve(jac @ jac.T + damping**2 * np.eye(6), step)
        q = np.clip(q + dq, kin.lower, kin.upper)
        if not np.all(np.isfinite(q)):
            raise ValueError('IK produced a non-finite joint target')
    return IkResult(q, False, pos_error, rot_error, max_iters)
