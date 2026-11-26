"""Pick and place geometry and sequences (plain Python, no ROS).

The Assignment 03 EEControl node takes fingertip targets in the robot's base frame (x forward, y left,
z up, metres) and always points the finger straight down. A pick is a list of phases, each a fingertip
target and a gripper value:

    target_b = object_in_base(robot_pose_map, object_xy_map)      # where the object is, seen from the robot
    phases = pick_sequence(target_b, settings)                    # approach, open, descend, grasp, lift
    unreachable(phases, ik, start)                                # check every target with the A03 IK first
    arm_reached(arm_state, command)                               # wait for each phase
    holding(gripper_reading, settings)                            # did the grasp catch the block?
    slot = free_slot(tray_pose, slots, other_objects, clearance)  # where on the tray to put it

Units: metres, radians in the map frame; joint angles in degrees and the gripper in percent, as on
/arm_state and /arm_commands (Assignment 03).
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


def unreachable(phases, ik, start_angles, max_error=0.005):
    """Phases whose fingertip target the IK cannot reach, as [(phase, error m)]. ik is the Assignment 03
    arm_control.kinematics.inverse_kinematics (or a stand-in in tests): ik(target, direction, start) ->
    (angles, error). Each phase starts from the previous solution, like the real arm. max_error is the
    IK node's own MAX_ERROR: it does not move for targets worse than that."""
    bad, angles = [], list(start_angles)
    for phase, target, _ in phases:
        solution, error = ik(list(target), [0.0, 0.0, -1.0], angles)
        if error > max_error:
            bad.append((phase, error))
        else:
            angles = list(solution)
    return bad


def arm_reached(arm_state, command, tolerance_deg=3.0):
    """True when the 5 arm joints of arm_state are within tolerance_deg of the command's 5 joint angles."""
    return all(abs(a - c) <= tolerance_deg for a, c in zip(arm_state[:5], command[:5]))


def holding(gripper_reading, s):
    """True if the gripper stopped on something: after closing it reads at least holding_min_gripper %
    (from manipulation.yaml) more than fully closed. Closed all the way means the block was missed."""
    return gripper_reading >= s["gripper_closed"] + s["holding_min_gripper"]


def free_slot(tray_pose, slots, others_xy, clearance):
    """First place on the tray with no other object within `clearance` m. tray_pose = (x, y, yaw) in the
    map; slots = [(dx, dy)] in the tray's own frame (manipulation.yaml); others_xy = map positions of the
    other objects. Returns the slot's map (x, y), or None when the tray is full."""
    tx, ty, yaw = tray_pose
    for dx, dy in slots:
        x = tx + math.cos(yaw) * dx - math.sin(yaw) * dy
        y = ty + math.sin(yaw) * dx + math.cos(yaw) * dy
        if all(math.dist((x, y), o) >= clearance for o in others_xy):
            return x, y
    return None


def with_grasp_offset(target_xy, shoulder_xy, offset):
    """Fingertip target for an object at target_xy (base frame). The IK places the fingertip (the end of
    the fixed jaw), but the object must end up between the jaws, so the target is shifted by
    offset = (radial, sideways) metres: radial away from the shoulder pan axis, sideways to its left.
    Measure the offset with try_pick on the real arm (manipulation.yaml, grasp_offset)."""
    dx, dy = target_xy[0] - shoulder_xy[0], target_xy[1] - shoulder_xy[1]
    r = math.hypot(dx, dy)
    ux, uy = (dx / r, dy / r) if r > 1e-9 else (1.0, 0.0)
    radial, sideways = offset
    return target_xy[0] + radial * ux - sideways * uy, target_xy[1] + radial * uy + sideways * ux
