"""Validation and uniform slow-down of MoveIt's timed joint trajectories.

Uniform time scaling preserves the quintic spline geometry. Sampling is a
discrete check, not a proof of continuous collision freedom or real tracking.
"""

from copy import deepcopy
import math


def seconds(duration):
    return duration.sec + duration.nanosec / 1e9


def set_seconds(duration, value):
    ns = round(value * 1e9)
    duration.sec, duration.nanosec = divmod(ns, 1_000_000_000)


def validate_trajectory(trajectory, joints):
    joint = trajectory.joint_trajectory
    if (len(joint.joint_names) != len(set(joint.joint_names)) or set(joint.joint_names) != set(joints)
            or trajectory.multi_dof_joint_trajectory.points):
        raise ValueError("trajectory_joint_mismatch")
    if len(joint.points) < 2:
        raise ValueError("empty_or_untimed_trajectory")
    previous = -1.0
    for point in joint.points:
        if any(len(values) != len(joints) for values in (point.positions, point.velocities, point.accelerations)):
            raise ValueError("trajectory_missing_derivatives")
        if not all(math.isfinite(v) for values in (point.positions, point.velocities, point.accelerations) for v in values):
            raise ValueError("nonfinite_trajectory")
        now = seconds(point.time_from_start)
        if now < 0 or now <= previous or not 0 <= point.time_from_start.nanosec < 1_000_000_000:
            raise ValueError("trajectory_time_not_increasing")
        previous = now
    if seconds(joint.points[0].time_from_start) > 1e-6:
        raise ValueError("trajectory_must_start_at_zero")
    if any(abs(v) > 1e-5 for point in (joint.points[0], joint.points[-1]) for v in point.velocities):
        raise ValueError("trajectory_endpoints_must_be_stationary")


def quintic(a, b, fraction):
    duration = seconds(b.time_from_start) - seconds(a.time_from_start)
    q, v, acceleration = [], [], []
    for q0, q1, v0, v1, a0, a1 in zip(a.positions, b.positions, a.velocities, b.velocities,
                                      a.accelerations, b.accelerations):
        x = q1 - q0 - v0 * duration - 0.5 * a0 * duration**2
        y = (v1 - v0 - a0 * duration) * duration
        z = (a1 - a0) * duration**2
        c = (q0, v0 * duration, 0.5 * a0 * duration**2,
             10*x - 4*y + z/2, -15*x + 7*y - z, 6*x - 3*y + z/2)
        q.append(sum(coef * fraction**i for i, coef in enumerate(c)))
        v.append(sum(i * c[i] * fraction**(i-1) for i in range(1, 6)) / duration)
        acceleration.append(sum(i*(i-1) * c[i] * fraction**(i-2) for i in range(2, 6)) / duration**2)
    return q, v, acceleration


def samples(trajectory, config):
    points = trajectory.joint_trajectory.points
    output = [(seconds(points[0].time_from_start), list(points[0].positions),
               list(points[0].velocities), list(points[0].accelerations))]
    for a, b in zip(points, points[1:]):
        duration = seconds(b.time_from_start) - seconds(a.time_from_start)
        distance = max(abs(x-y) for x, y in zip(a.positions, b.positions))
        count = max(2, math.ceil(duration/config.validation_time_step),
                    math.ceil(distance/config.validation_joint_step))
        if len(output) + count > config.max_validation_samples:
            raise ValueError("trajectory_validation_sample_limit")
        for i in range(1, count+1):
            q, v, acc = quintic(a, b, i/count)
            output.append((seconds(a.time_from_start) + duration*i/count, q, v, acc))
    return output


def slow_down(trajectory, factor):
    if not math.isfinite(factor) or factor < 1:
        raise ValueError("invalid_time_scaling")
    result = deepcopy(trajectory)
    # Zero header => start on acceptance, never reuse a planning-time schedule.
    result.joint_trajectory.header.stamp.sec = 0
    result.joint_trajectory.header.stamp.nanosec = 0
    for point in result.joint_trajectory.points:
        set_seconds(point.time_from_start, seconds(point.time_from_start) * factor)
        point.velocities = [v / factor for v in point.velocities]
        point.accelerations = [a / factor**2 for a in point.accelerations]
    return result
