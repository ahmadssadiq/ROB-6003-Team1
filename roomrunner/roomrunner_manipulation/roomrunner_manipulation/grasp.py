"""Pick and place geometry and sequences (plain Python, no ROS).

The Assignment 03 EEControl node takes fingertip targets in the robot's base frame (x forward, y left,
z up, metres) and always points the finger straight down. A pick is a list of phases, each a fingertip
target and a gripper value:

    target_b = object_in_base(robot_pose_map, object_xy_map)      # where the object is, seen from the robot
    phases = pick_sequence(target_b, settings)                    # approach, open, descend, grasp, lift
"""
import math


def object_in_base(robot_pose, object_xy):
    """An object's (x, y) in the map frame -> (x, y) in the robot's base frame. robot_pose = (x, y, yaw) in map."""
    rx, ry, yaw = robot_pose
    dx, dy = object_xy[0] - rx, object_xy[1] - ry
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


def in_reach(target_xy, shoulder_xy, min_reach, max_reach):
    """True if the target is within the arm's horizontal reach, measured from the shoulder pan axis."""
    r = math.dist(target_xy, shoulder_xy)
    return min_reach <= r <= max_reach


def pick_sequence(target_xy, s):
    """Phases to pick an object at target_xy (base frame). s: dict with grasp_z, hover_height, lift_height,
    gripper_open, gripper_closed (see config/manipulation.yaml). Returns [(phase, (x, y, z), gripper %)]."""
    x, y = target_xy
    hover, grasp, lift = s["grasp_z"] + s["hover_height"], s["grasp_z"], s["grasp_z"] + s["lift_height"]
    return [("approach", (x, y, hover), s["gripper_open"]),
            ("descend", (x, y, grasp), s["gripper_open"]),
            ("grasp", (x, y, grasp), s["gripper_closed"]),
            ("lift", (x, y, lift), s["gripper_closed"])]


def place_sequence(target_xy, s):
    """Phases to put the held object down at target_xy (base frame) and move clear."""
    x, y = target_xy
    hover, release = s["place_z"] + s["hover_height"], s["place_z"]
    return [("carry", (x, y, hover), s["gripper_closed"]),
            ("lower", (x, y, release), s["gripper_closed"]),
            ("release", (x, y, release), s["gripper_open"]),
            ("retreat", (x, y, hover), s["gripper_open"])]
