"""Shared orientation semantics for seeds, numerical IK, and final validation.

Copied from g1_pointcloud_grasping genie_motion/pose_task.py (G1-only 5D scoring removed)."""
import math

import numpy as np

from .kinematics import rotation_vector, rotation_quaternion


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def right_log_inverse(vector):
    theta = float(np.linalg.norm(vector))
    coefficient = (1/12+theta*theta/720 if theta < 1e-4 else
                   (1-0.5*theta/math.tan(theta/2))/(theta*theta))
    cross = skew(vector)
    return np.eye(3)+0.5*cross+coefficient*(cross@cross)


def orientation_residual(current, target, mode='full'):
    if mode == 'full':
        return rotation_vector(current, target)
    if mode == 'tool_z':
        # Three components with rank two: a unit axis has only two DOF.
        # The tool's X/Y directions (spin about its own Z) are intentionally free.
        return target[:, 2]-current[:, 2]
    raise ValueError(f'Unknown orientation constraint mode: {mode}')


def orientation_distance(current, target, mode='full'):
    if mode == 'full':
        return float(np.linalg.norm(rotation_vector(current, target)))
    if mode == 'tool_z':
        a, b = current[:, 2], target[:, 2]
        # Distinguishes parallel and antiparallel; cross alone cannot do that.
        return math.atan2(float(np.linalg.norm(np.cross(a, b))), float(np.clip(a@b, -1., 1.)))
    raise ValueError(f'Unknown orientation constraint mode: {mode}')


def orientation_jacobian(current, target, angular_jacobian, mode='full'):
    """A for e(q+dq) ~= e(q) - A*dq, in the same root frame as KDL."""
    if mode == 'full':
        return right_log_inverse(rotation_vector(current, target))@angular_jacobian
    if mode == 'tool_z':
        # d(axis)/dq = omega x axis = -[axis]x * J_omega.
        return -skew(current[:, 2])@angular_jacobian
    raise ValueError(f'Unknown orientation constraint mode: {mode}')


def task_nullspace(kin, q, target_rotation, mode='full'):
    """Columns span the actual task kernel, including rank loss at singularities."""
    _, rotation = kin.fk(q)
    jacobian = kin.jacobian(q)
    task = np.vstack((jacobian[:3], 0.2*orientation_jacobian(
        rotation, target_rotation, jacobian[3:], mode)))
    _, singular_values, vt = np.linalg.svd(task, full_matrices=True)
    threshold = max(1e-9, 1e-6*float(singular_values[0]))
    rank = int(np.count_nonzero(singular_values > threshold))
    return vt[rank:].T, rank


# G2 port: the 5D (tool_z) travel weights and Link6 axial-spin score in genie_motion are keyed by
# G1 joint/link names (Joint6_l, Link6_r ...) and were not ported; only full 6-D mode exists here.
def travel_weights(kin, mode='full'):
    """Unweighted joint travel (genie_motion's 'full' mode behaviour)."""
    if mode != 'full':
        raise ValueError(f'G2 ik_lab only supports orientation_mode=full, got {mode}')
    return np.ones(kin.n)
