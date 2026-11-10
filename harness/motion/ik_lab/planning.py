"""Shared candidate validation, path metrics and JTC time parameterization.

Copied from g1_pointcloud_grasping genie_motion/planning.py (5D scoring branch removed)."""
from dataclasses import dataclass
import math

import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .pose_task import orientation_distance, travel_weights


@dataclass(frozen=True)
class MotionSettings:
    velocity: float
    acceleration: float
    jerk: float
    travel_warning_deg: float


@dataclass(frozen=True)
class PlannedMotion:
    trajectory: JointTrajectory
    duration: float
    label: str


def make_trajectory(path, joint_names, velocity_limits, settings):
    trajectory = JointTrajectory(joint_names=list(joint_names))
    # Each segment is monotonic quintic with zero endpoint velocity/acceleration.
    # Interior waypoints are stops, not a claim of non-stop trajectory smoothing.
    times = [0.0]
    for q0, q1 in zip(path[:-1], path[1:]):
        delta = np.abs(q1-q0)
        distance = float(np.max(delta))
        velocity = np.minimum(velocity_limits, settings.velocity)
        duration = max(1.0, float(np.max(1.875*delta/velocity)),
                       math.sqrt(5.774*distance/settings.acceleration),
                       (60*distance/settings.jerk)**(1/3))
        times.append(times[-1]+duration)
    for q, t in zip(path, times):
        point = JointTrajectoryPoint(positions=q.tolist(), velocities=[0.0]*len(q),
                                     accelerations=[0.0]*len(q))
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(t*1e9), 10**9)
        trajectory.points.append(point)
    return trajectory, times[-1]


def plan_motion(algorithm, request, velocity_limits, settings):
    cost_mode = algorithm.cost_mode or request.orientation_mode
    ranked = []
    for candidate in algorithm.plan(request):
        request.check_deadline()
        path = np.asarray(candidate.waypoints, dtype=float)
        kin = request.kin
        if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] != kin.n or not np.all(np.isfinite(path)):
            request.log(f'{candidate.label} rejected: invalid path shape or values')
            continue
        if np.max(np.abs(path[0]-request.start)) > 1e-6:
            request.log(f'{candidate.label} rejected: path start does not equal the planning start')
            continue
        if np.any(path < kin.lower) or np.any(path > kin.upper):
            request.log(f'{candidate.label} rejected: path joints out of limits')
            continue
        p, rot = kin.fk(path[-1])
        if (np.linalg.norm(p-request.goal_position) > 1e-4 or
                orientation_distance(rot, request.goal_rotation, request.orientation_mode) > 1e-3):
            request.log(f'{candidate.label} rejected: endpoint FK does not reach target precision')
            continue
        travel = np.sum(np.abs(np.diff(path, axis=0)), axis=0)
        degrees = np.degrees(travel)
        request.log(f'{candidate.label} planned joint cumulative travel °={np.round(degrees, 1)}, '
                    f'total={np.sum(degrees):.1f}°, max single joint={np.max(degrees):.1f}°')
        if np.max(degrees) > settings.travel_warning_deg:
            request.log(f'{candidate.label} travel note: exceeds the {settings.travel_warning_deg:.1f}° reference; '
                        'recorded only, not rejected for travel; continuing to scoring and collision checks')
        scored_travel = travel*travel_weights(kin, cost_mode)
        score = float(np.sum(scored_travel)+0.5*np.max(scored_travel))
        ranked.append((score, candidate.label, path.copy()))
    ranked.sort(key=lambda item: item[0])
    for score, label, path in ranked:
        valid = True
        for q0, q1 in zip(path[:-1], path[1:]):
            count = max(1, math.ceil(np.max(np.abs(q1-q0))/math.radians(2)))
            for s in np.linspace(0, 1, count+1):
                request.check_deadline()
                if not request.check_state(q0+s*(q1-q0)):
                    valid = False
                    break
            if not valid:
                break
        if not valid:
            request.log(f'{label} connecting path collision check failed; trying the next candidate')
            continue
        trajectory, duration = make_trajectory(path, request.kin.joint_names, velocity_limits, settings)
        request.check_deadline()
        request.log(f'Selected {label}, score={score:.4f}, waypoints={len(path)}')
        return PlannedMotion(trajectory, duration, label)
    raise ValueError('The algorithm provided no full path passing common acceptance; no path prefix sent')
