"""A small simulation of the LeKiwi base for testing and tuning the follower offline (plain Python).

The model, per step of dt seconds:
    the wheels follow the command with a first-order lag (time constant `lag` s),
    below `deadband` m/s (and deadband_turn rad/s) the wheels do not move,
    the pose the controller sees is `pose_delay` s old.
The follower loop is the same as waypoint_follower.py's (controller.py functions).

    python3 -m roomrunner_control.sim_base            # compare a few gain settings on a test route
"""
import math

from roomrunner_control.controller import (at_goal, limit_acceleration, next_index, velocity_command,
                                           with_min_speed, wrap)

FOLLOWER = dict(max_speed=0.15, max_turn=0.5, gain=1.5, yaw_gain=1.5, lookahead=0.15, min_speed=0.03,
                max_accel=0.3, max_turn_accel=1.5)
BASE = dict(lag=0.15, deadband=0.02, deadband_turn=0.05, pose_delay=0.1)
ROUTE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.2)]       # an L-shaped path with a final bend


def simulate(points, goal_yaw, follower=FOLLOWER, base=BASE, start=(0.0, 0.0, 0.0), dt=0.05, timeout=60.0,
             position_tolerance=0.05, yaw_tolerance=math.radians(5), settle_time=0.5):
    """Drive along points (map frame) to goal_yaw. Returns a dict: reached, time (s), final_error (m),
    final_yaw_error (rad), path_length (m driven), max_speed (m/s), trajectory [(t, x, y, yaw)]."""
    f, b = follower, base
    x, y, yaw = start
    actual = command = (0.0, 0.0, 0.0)
    delay_steps = max(0, round(b["pose_delay"] / dt))
    seen = [start] * (delay_steps + 1)
    index, settled, t, driven, top, trajectory = 0, 0.0, 0.0, 0.0, 0.0, []
    alpha = dt / (b["lag"] + dt)
    while t < timeout:
        pose = seen[0]                                   # what the controller sees (delayed)
        index = next_index(points, index, pose[:2], f["lookahead"])
        if index == len(points) - 1 and at_goal(pose, points[-1], goal_yaw, position_tolerance, yaw_tolerance):
            target = (0.0, 0.0, 0.0)
            settled += dt
            if settled >= settle_time:
                break
        else:
            settled = 0.0
            target = velocity_command(pose, points[index], goal_yaw, f["max_speed"], f["gain"], f["max_turn"],
                                      f["yaw_gain"])
            target = with_min_speed(target, f["min_speed"])
        command = limit_acceleration(command, target, f["max_accel"], f["max_turn_accel"], dt)

        # the base: lag, then deadband
        actual = tuple(a + alpha * (c - a) for a, c in zip(actual, command))
        forward, left, turn = actual
        if math.hypot(forward, left) < b["deadband"]:
            forward = left = 0.0
        if abs(turn) < b["deadband_turn"]:
            turn = 0.0
        x += (math.cos(yaw) * forward - math.sin(yaw) * left) * dt
        y += (math.sin(yaw) * forward + math.cos(yaw) * left) * dt
        yaw = wrap(yaw + turn * dt)
        driven += math.hypot(forward, left) * dt
        top = max(top, math.hypot(forward, left))
        t += dt
        seen = seen[1:] + [(x, y, yaw)]
        trajectory.append((t, x, y, yaw))
    return {"reached": t < timeout, "time": t, "final_error": math.dist((x, y), points[-1]),
            "final_yaw_error": wrap(goal_yaw - yaw), "path_length": driven, "max_speed": top,
            "trajectory": trajectory}


def main():
    print("gain  max_speed  min_speed | reached  time s  error cm  driven m")
    for gain, speed, min_speed in [(1.0, 0.15, 0.03), (1.5, 0.15, 0.03), (2.5, 0.15, 0.03), (1.5, 0.25, 0.03),
                                   (1.5, 0.15, 0.0)]:
        r = simulate(ROUTE, math.pi / 2, dict(FOLLOWER, gain=gain, max_speed=speed, min_speed=min_speed))
        print(f"{gain:4.1f}  {speed:9.2f}  {min_speed:9.2f} | {str(r['reached']):7s}  {r['time']:6.1f}  "
              f"{100 * r['final_error']:8.1f}  {r['path_length']:8.2f}")


if __name__ == "__main__":
    main()
