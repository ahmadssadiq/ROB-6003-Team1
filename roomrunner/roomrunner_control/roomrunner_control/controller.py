"""Path following for an omnidirectional base (plain Python, no ROS).

The LeKiwi base can drive in any direction while it turns, so the controller drives straight at the
current waypoint and turns towards the goal heading at the same time:

    v_map   = min(v_max, k * distance) towards the waypoint       (proportional, saturated)
    v_body  = R(-yaw) v_map                                         (map -> robot frame)
    omega   = clip(k_yaw * wrap(goal_yaw - yaw), -omega_max, omega_max)
"""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def next_index(points, index, position, lookahead):
    """Move on past waypoints that are already within `lookahead` (never past the last one)."""
    while index < len(points) - 1 and math.dist(position, points[index]) < lookahead:
        index += 1
    return index


def velocity_command(pose, waypoint, goal_yaw, max_speed, gain, max_turn, yaw_gain):
    """Body-frame command (forward m/s, left m/s, turn rad/s) from the robot pose (x, y, yaw) in the map."""
    x, y, yaw = pose
    dx, dy = waypoint[0] - x, waypoint[1] - y
    distance = math.hypot(dx, dy)
    speed = min(max_speed, gain * distance)
    vx, vy = (speed * dx / distance, speed * dy / distance) if distance > 1e-9 else (0.0, 0.0)
    forward = math.cos(yaw) * vx + math.sin(yaw) * vy
    left = -math.sin(yaw) * vx + math.cos(yaw) * vy
    turn = max(-max_turn, min(max_turn, yaw_gain * wrap(goal_yaw - yaw)))
    return forward, left, turn


def at_goal(pose, goal, goal_yaw, position_tolerance, yaw_tolerance):
    return (math.dist(pose[:2], goal) < position_tolerance
            and abs(wrap(goal_yaw - pose[2])) < yaw_tolerance)


def remaining_distance(position, points, index):
    """Distance to the current waypoint plus the rest of the path after it."""
    return math.dist(position, points[index]) + sum(math.dist(points[i], points[i + 1])
                                                    for i in range(index, len(points) - 1))


def with_min_speed(command, min_speed):
    """Raise a small, non-zero drive command to min_speed (same direction). The motors do not move below a
    certain speed, so without this the robot can stall just short of the goal, outside the tolerance."""
    forward, left, turn = command
    speed = math.hypot(forward, left)
    if 1e-9 < speed < min_speed:
        forward, left = forward * min_speed / speed, left * min_speed / speed
    return forward, left, turn


def limit_acceleration(previous, command, max_accel, max_turn_accel, dt):
    """Change the (forward, left, turn) command by at most max_accel (m/s^2) and max_turn_accel (rad/s^2)
    per step of dt seconds. Smooth starts and stops slip less, so odometry stays better."""
    def step(old, new, limit):
        return old + max(-limit * dt, min(limit * dt, new - old))
    (f0, l0, t0), (f1, l1, t1) = previous, command
    df, dl = f1 - f0, l1 - l0
    change = math.hypot(df, dl)
    if change > max_accel * dt:                      # scale the linear change, keep its direction
        df, dl = df * max_accel * dt / change, dl * max_accel * dt / change
    return f0 + df, l0 + dl, step(t0, t1, max_turn_accel)


def overhead_correction(goal, goal_yaw, pf_pose, overhead_pose):
    """Goal to drive to (in the filter's map pose) so that the robot ends at `goal` as the overhead camera
    sees it. Both poses are taken with the robot standing still; their difference is the filter's local error:
        corrected goal = goal + (pf - overhead),   corrected yaw = goal_yaw + (pf_yaw - overhead_yaw)
    Returns ((x, y), yaw, error), error = the overhead camera's distance (m) from the robot to the goal."""
    dx, dy = pf_pose[0] - overhead_pose[0], pf_pose[1] - overhead_pose[1]
    dyaw = wrap(pf_pose[2] - overhead_pose[2])
    error = math.dist(overhead_pose[:2], goal)
    return (goal[0] + dx, goal[1] + dy), wrap(goal_yaw + dyaw), error
