"""Checking the particle filter against the overhead camera (plain Python, no ROS).

Two frames (docs/frames.md): the particle filter works in aruco_map (Assignment 04, wall markers); the
overhead camera reports the robot-top tag in map. p_map = T_map_aruco p_aruco_map.

    T, errors = fit_T_map_aruco(map_poses, aruco_poses)     # measure T_map_aruco once (measure_t_map_aruco)
    monitor = PoseMonitor()                                  # during runs (pose_monitor node)
    action = monitor.check(t, pf_pose_in_map, overhead_pose_in_map, robot_still)

All poses are (x, y, yaw) in metres and radians.
"""
import math

import numpy as np

from roomrunner_localization.transform2d import fit_rigid, pose, transform_pose


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def average_pose(poses):
    """Mean of several (x, y, yaw); the yaw is averaged on the circle."""
    p = np.asarray(poses, float).reshape(-1, 3)
    return (float(p[:, 0].mean()), float(p[:, 1].mean()),
            math.atan2(np.sin(p[:, 2]).mean(), np.cos(p[:, 2]).mean()))


def fit_T_map_aruco(map_poses, aruco_poses):
    """T_map_aruco from the robot's pose seen in both frames at several spots (at least 3, spread out).

    The translation and rotation come from the positions (fit_rigid); the headings are only checked.
    Returns (T, errors), errors[i] = (distance m, heading rad) between map_poses[i] and the fitted
    T_map_aruco aruco_poses[i]. A few cm and a few degrees is good; more means a bad spot or a bad tag."""
    if len(map_poses) < 3:
        raise ValueError("need at least 3 spots")
    T = fit_rigid([p[:2] for p in map_poses], [p[:2] for p in aruco_poses])
    errors = []
    for m, a in zip(map_poses, aruco_poses):
        x, y, yaw = transform_pose(T, a)
        errors.append((math.hypot(x - m[0], y - m[1]), wrap(yaw - m[2])))
    return T, errors


class PoseHistory:
    """The last few seconds of (time, pose), to look up where the filter was when an overhead frame was
    taken, and whether the robot was standing still then."""

    def __init__(self, seconds=5.0):
        self.seconds, self.samples = seconds, []

    def add(self, t, pose_):
        self.samples.append((t, pose_))
        while self.samples and self.samples[0][0] < t - self.seconds:
            self.samples.pop(0)

    def nearest(self, t):
        if not self.samples:
            return None
        return min(self.samples, key=lambda s: abs(s[0] - t))[1]

    def still(self, t, window=1.0, max_speed=0.05, max_turn=0.1):
        """True if the robot moved less than max_speed (m/s) and turned less than max_turn (rad/s) in the
        window seconds before t."""
        near = [(s, p) for s, p in self.samples if t - window <= s <= t]
        if len(near) < 2:
            return False
        (t0, p0), (t1, p1) = near[0], near[-1]
        dt = max(t1 - t0, 1e-3)
        moved, turned = math.hypot(p1[0] - p0[0], p1[1] - p0[1]), abs(wrap(p1[2] - p0[2]))
        return moved / dt <= max_speed and turned / dt <= max_turn and t1 - t0 >= 0.5 * window


class PoseMonitor:
    """Decides when the particle filter has lost track: its pose and the overhead tag disagree by more than
    max_distance or max_heading on `strikes` checks in a row (the robot standing still each time).

    check() returns "skip" (robot moving, nothing compared), "ok", "disagree" (not yet enough strikes,
    or a reset happened less than cooldown seconds ago) or "reset" (re-seed the filter at the overhead pose).

    The defaults are above 0.3 m because the Assignment 04 filter spreads its particles 0.3 m around a
    re-seed pose, so right after a reset it can be about that far off until it sees a wall marker."""

    def __init__(self, max_distance=0.40, max_heading=math.radians(25), strikes=2, cooldown=30.0):
        self.max_distance, self.max_heading = max_distance, max_heading
        self.strikes, self.cooldown = strikes, cooldown
        self.count, self.last_reset = 0, None

    def check(self, t, pf_pose, overhead_pose, still=True):
        if not still or pf_pose is None:
            return "skip"
        distance = math.hypot(pf_pose[0] - overhead_pose[0], pf_pose[1] - overhead_pose[1])
        heading = abs(wrap(pf_pose[2] - overhead_pose[2]))
        if distance <= self.max_distance and heading <= self.max_heading:
            self.count = 0
            return "ok"
        self.count += 1
        cooling = self.last_reset is not None and t - self.last_reset < self.cooldown
        if self.count >= self.strikes and not cooling:
            self.count, self.last_reset = 0, t
            return "reset"
        return "disagree"


def T_to_yaml(T, errors):
    """The T_map_aruco.yaml content for a fitted transform, with the fit's check numbers."""
    x, y, yaw = pose(T)
    worst = max(e[0] for e in errors)
    return {"T_map_aruco": {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)}, "measured": True,
            "spots": len(errors), "worst_error_m": round(worst, 4)}
