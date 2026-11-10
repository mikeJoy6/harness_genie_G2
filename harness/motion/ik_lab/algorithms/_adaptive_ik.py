"""Pose-only adaptive damped least squares with actual-error backtracking.

This is a local numerical IK solver, not a path/collision planner. The old DIK
solver remains unchanged. See multi_ik_adaptive.md for equations and parameters.
"""
import time

import numpy as np

from ..kinematics import IkResult
from ..pose_task import orientation_residual, orientation_distance, orientation_jacobian


def solve_adaptive(kin, seed, position, rotation, max_iters, deadline, orientation_mode='full',
                   *, position_tolerance=1e-4, rotation_tolerance=1e-3):
    # Full position/orientation error is used. No fixed 2 mm residual clipping.
    angular_weight = 0.2  # meters/radian, an explicit task-scale choice
    damping, radius = 0.01, 0.25  # numerical regularization, joint step in rad
    q = np.clip(np.array(seed, dtype=float, copy=True), kin.lower, kin.upper)
    accepted, rejected, consecutive_rejects = 0, 0, 0
    started = time.monotonic()

    def evaluate(values):
        p, r = kin.fk(values)
        ep, er = position-p, orientation_residual(r, rotation, orientation_mode)
        error = np.concatenate((ep, angular_weight*er))
        if not np.all(np.isfinite(error)):
            raise ValueError('Adaptive IK FK/error is not finite')
        return ep, er, error, 0.5*float(error@error), r

    ep, er, error, cost, current_rotation = evaluate(q)

    def finish(reason, iterations):
        return IkResult(q.copy(), reason == 'converged', float(np.linalg.norm(ep)),
                        orientation_distance(current_rotation, rotation, orientation_mode), iterations, reason,
                        f'accepted={accepted},rejected={rejected},damping={damping:.3g},'
                        f'joint step radius={radius:.3g}rad,time={(time.monotonic()-started)*1000:.2f}ms')

    for iteration in range(max_iters+1):
        # Same endpoint acceptance criteria as the original solver/public layer.
        if (np.linalg.norm(ep) < position_tolerance and
                orientation_distance(current_rotation, rotation, orientation_mode) < rotation_tolerance):
            return finish('converged', iteration)
        if time.monotonic() >= deadline:
            return finish('ik_deadline', iteration)
        if iteration == max_iters:
            return finish('iteration_limit', iteration)
        jac = kin.jacobian(q)
        # Full pose uses Log + Jr inverse; 5D uses the rank-two axis derivative.
        task_jac = np.vstack((jac[:3], angular_weight*orientation_jacobian(
            current_rotation, rotation, jac[3:], orientation_mode)))
        gradient = task_jac.T@error
        # Freeze joints at a bound only when the descent direction points out.
        active = (((q <= kin.lower+1e-10) & (gradient < 0)) |
                  ((q >= kin.upper-1e-10) & (gradient > 0)))
        free = np.flatnonzero(~active)
        if free.size == 0 or np.linalg.norm(gradient[free], ord=np.inf) < 1e-12:
            return finish('constrained_stationary', iteration)
        reduced = task_jac[:, free]
        delta = np.zeros(kin.n)
        try:
            delta[free] = np.linalg.solve(reduced.T@reduced+damping*damping*np.eye(len(free)),
                                          gradient[free])
        except np.linalg.LinAlgError:
            return finish('linear_solve_failed', iteration)
        if not np.all(np.isfinite(delta)):
            return finish('nonfinite_step', iteration)
        largest = float(np.max(np.abs(delta)))
        if largest > radius:
            delta *= radius/largest
        improved = False
        # Test actual nonlinear FK residual, not only a linear prediction.
        for backtrack in range(8):
            if time.monotonic() >= deadline:
                return finish('ik_deadline', iteration)
            trial = np.clip(q+delta*(0.5**backtrack), kin.lower, kin.upper)
            actual_step = trial-q
            if np.max(np.abs(actual_step)) < 1e-12:
                break
            next_ep, next_er, next_error, next_cost, next_rotation = evaluate(trial)
            predicted_error = error-task_jac@actual_step
            predicted_drop = cost-0.5*float(predicted_error@predicted_error)
            actual_drop = cost-next_cost
            if actual_drop > 1e-14*max(1.0, cost):
                ratio = actual_drop/predicted_drop if predicted_drop > 1e-16 else 0.0
                q, ep, er, error, cost, current_rotation = trial, next_ep, next_er, next_error, next_cost, next_rotation
                accepted += 1
                consecutive_rejects = 0
                if ratio > 0.75:
                    damping = max(1e-6, damping*0.5)
                    radius = min(0.6, radius*1.5)
                elif ratio < 0.25:
                    damping = min(10.0, damping*2.0)
                    radius = max(1e-6, radius*0.7)
                improved = True
                break
            rejected += 1
        if not improved:
            consecutive_rejects += 1
            damping = min(10.0, damping*4.0)
            radius = max(1e-6, radius*0.5)
            if consecutive_rejects >= 6:
                return finish('stalled', iteration+1)
    raise RuntimeError('unreachable')
