"""Closed-loop checks of the follower logic on the simulated base (sim_base.py)."""
import math

import pytest

from roomrunner_control.controller import limit_acceleration, overhead_correction, with_min_speed
from roomrunner_control.sim_base import BASE, FOLLOWER, ROUTE, simulate


def test_follows_the_route_and_stops_aligned():
    r = simulate(ROUTE, math.pi / 2)
    assert r["reached"] and r["final_error"] < 0.05 and abs(r["final_yaw_error"]) < math.radians(5)
    assert r["path_length"] < 1.2 * 3.24                # close to the path's own length, no big detours
    assert r["max_speed"] <= FOLLOWER["max_speed"] + 1e-6


def test_min_speed_beats_a_motor_deadband():
    sticky = dict(BASE, deadband=0.08)                   # motors that need 8 cm/s to move at all
    stalls = simulate([(0, 0), (1.0, 0)], 0.0, dict(FOLLOWER, min_speed=0.0), sticky, timeout=30,
                      position_tolerance=0.02)
    arrives = simulate([(0, 0), (1.0, 0)], 0.0, dict(FOLLOWER, min_speed=0.09), sticky, timeout=30,
                       position_tolerance=0.02)
    assert not stalls["reached"] and stalls["final_error"] > 0.02       # stuck 2 to 3 cm short
    assert arrives["reached"] and arrives["final_error"] < 0.02


def test_acceleration_is_limited():
    """Reaching 90 % of the top speed takes at least 0.9 max_speed / max_accel seconds."""
    r = simulate([(0, 0), (1.0, 0)], 0.0)
    traj = r["trajectory"]
    speeds = [(b[0], math.dist(a[1:3], b[1:3]) / (b[0] - a[0])) for a, b in zip(traj, traj[1:])]
    rise = next(t for t, v in speeds if v >= 0.9 * FOLLOWER["max_speed"])
    assert rise >= 0.9 * FOLLOWER["max_speed"] / FOLLOWER["max_accel"]


def test_with_min_speed():
    assert with_min_speed((0.01, 0.0, 0.2), 0.03) == pytest.approx((0.03, 0.0, 0.2))
    assert with_min_speed((0.0, 0.0, 0.2), 0.03) == (0.0, 0.0, 0.2)       # standing: only turning
    assert with_min_speed((0.1, 0.0, 0.0), 0.03) == (0.1, 0.0, 0.0)


def test_limit_acceleration_keeps_direction():
    f, l, t = limit_acceleration((0, 0, 0), (0.3, 0.4, 2.0), max_accel=1.0, max_turn_accel=1.0, dt=0.1)
    assert (f, l, t) == pytest.approx((0.06, 0.08, 0.1))


def test_overhead_correction():
    # the filter says (1.00, 1.00) but the overhead camera sees the robot at (1.04, 0.98), 3 deg off
    goal, yaw, error = overhead_correction((1.0, 1.0), 0.0, (1.0, 1.0, 0.0), (1.04, 0.98, math.radians(3)))
    assert goal == pytest.approx((0.96, 1.02)) and yaw == pytest.approx(math.radians(-3))
    assert error == pytest.approx(math.hypot(0.04, 0.02))
