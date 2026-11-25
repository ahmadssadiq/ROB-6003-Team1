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
