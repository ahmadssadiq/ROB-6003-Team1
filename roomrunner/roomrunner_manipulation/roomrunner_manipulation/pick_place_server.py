"""Pick and place action server, on top of the Assignment 03 EEControl node (inverse_kinematics).

Action server:
    pick_place     roomrunner_interfaces/PickPlace: mode ("pick" or "place"), object name, target pose (map)
Subscribes:
    robot_pose     geometry_msgs/PoseStamped (map): where the base is, to turn the target into the base frame
    arm_state      std_msgs/Float64MultiArray: 5 joint angles (deg) and the gripper (%), from robot_node
    arm_commands   std_msgs/Float64MultiArray: what the IK node sent, to know it accepted a target
    objects        roomrunner_interfaces/ObjectPoseArray (map): to find a free spot on the tray
Publishes:
    ik_target      geometry_msgs/PoseStamped (base frame): fingertip targets for the Assignment 03 IK node
Parameters:
    mode           "stub" (default: report phases, move nothing) or "real"
    settings_file  manipulation.yaml (heights, gripper values, reach, timings, tray slots)
    phase_time     s per phase in stub mode (default 0.5)
    stub_fail_rate fraction of stub picks that report a missed grasp, to try the mission's RETRY (default 0)

In real mode every arm command goes through the Assignment 03 IK node, the gripper too: before a phase
that changes the gripper, this node sets the IK node's own "gripper" parameter, then sends the target.
So only one node ever publishes /arm_commands.

Checks in real mode (grasp.py): every phase target is tried with the Assignment 03 IK function before the
arm moves (fails early when out of reach); each phase waits until arm_state reaches the IK node's command
(or phase_timeout); after the lift, the gripper reading tells whether the block is held, otherwise the
pick fails with "missed" and the mission FSM retries. A place on a tray goes to the first free slot.
"""
import math
import random
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Float64MultiArray

from roomrunner_interfaces.action import PickPlace
from roomrunner_interfaces.msg import ObjectPoseArray
from roomrunner_manipulation.grasp import (arm_reached, free_slot, holding, in_reach, object_in_base,
                                           pick_sequence, place_sequence, unreachable, with_grasp_offset)

STUB_PHASES = {"pick": ["approach", "descend", "grasp", "lift"], "place": ["carry", "lower", "release", "retreat"]}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_ik():
    """The Assignment 03 inverse_kinematics function, or None if it cannot be imported here."""
    try:
        from arm_control.kinematics import inverse_kinematics
        return inverse_kinematics
    except ImportError:
        return None


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__("pick_place_server")
        self.declare_parameter("mode", "stub")
        self.declare_parameter("settings_file", "")
        self.declare_parameter("phase_time", 0.5)
        self.declare_parameter("stub_fail_rate", 0.0)
        self.mode = self.get_parameter("mode").value
        self.settings = self.load_settings()
        group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.robot = self.arm_state = None
        self.last_command, self.command_time = None, 0.0
        self.objects = {}
        self.held = None
        self.create_subscription(PoseStamped, "robot_pose", self.on_pose, 10, callback_group=group)
        self.create_subscription(Float64MultiArray, "arm_state", self.on_arm_state, 10, callback_group=group)
        self.create_subscription(Float64MultiArray, "arm_commands", self.on_arm_command, 10, callback_group=group)
        self.create_subscription(ObjectPoseArray, "objects", self.on_objects, 10, callback_group=group)
        self.target_pub = self.create_publisher(PoseStamped, "ik_target", 10)
        self.ik_params = AsyncParameterClient(self, "inverse_kinematics", callback_group=group)
        self.gripper = None
        self.ik = load_ik() if self.mode == "real" else None
        if self.mode == "real" and self.ik is None:
            self.get_logger().warn("Cannot import arm_control.kinematics: targets are not checked before moving.")
        self.server = ActionServer(self, PickPlace, "pick_place", self.execute, callback_group=group)
        self.get_logger().info(f"Pick and place server ready ({self.mode} mode).")

    def load_settings(self):
        """manipulation.yaml, read again for every goal so edits apply without a restart (try_pick)."""
        path = self.get_parameter("settings_file").value
        if not path:
            return {}
        with open(path) as f:
            return yaml.safe_load(f)

    def on_pose(self, msg):
        self.robot = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))

    def on_arm_state(self, msg):
        with self.lock:
            self.arm_state = list(msg.data)

    def on_arm_command(self, msg):
        with self.lock:
            self.last_command, self.command_time = list(msg.data), time.monotonic()

    def on_objects(self, msg):
        self.objects = {o.name: (o.kind, o.pose.position.x, o.pose.position.y, yaw_of(o.pose.orientation))
                        for o in msg.objects}

    def execute(self, goal_handle):
        goal = goal_handle.request
        result = PickPlace.Result()
        if goal.mode not in STUB_PHASES:
            goal_handle.abort()
            result.message = f"unknown mode {goal.mode!r}"
            return result
        try:
            self.settings = self.load_settings()
            if self.mode == "stub":
                self.run_stub(goal_handle, goal)
            else:
                self.run_real(goal_handle, goal)
        except RuntimeError as error:
            goal_handle.abort()
            result.message = str(error)
            self.get_logger().warn(f"{goal.mode} {goal.object_name} failed: {error}")
            return result
        self.held = goal.object_name if goal.mode == "pick" else None
        goal_handle.succeed()
        result.success = True
        result.message = f"{goal.mode} {goal.object_name} done" + (" (stub)" if self.mode == "stub" else "")
        return result

    def run_stub(self, goal_handle, goal):
        for phase in STUB_PHASES[goal.mode]:
            goal_handle.publish_feedback(PickPlace.Feedback(phase=phase))
            time.sleep(self.get_parameter("phase_time").value)
        if goal.mode == "pick" and random.random() < self.get_parameter("stub_fail_rate").value:
            raise RuntimeError(f"missed {goal.object_name} (stub_fail_rate)")

    def place_xy(self, goal):
        """Where to put the held object down (map): a free slot when placing on a tray, else the target."""
        target = (goal.target.pose.position.x, goal.target.pose.position.y)
        tray = self.objects.get(goal.object_name)
        if goal.mode != "place" or tray is None or tray[0] != "tray" or not self.settings.get("tray_slots"):
            return target
        others = [(x, y) for name, (kind, x, y, _) in self.objects.items() if name not in (goal.object_name, self.held)]
        slot = free_slot(tray[1:], self.settings["tray_slots"], others, self.settings["slot_clearance"])
        if slot is None:
            raise RuntimeError(f"{goal.object_name} is full")
        return slot

    def run_real(self, goal_handle, goal):
        s = self.settings
        with self.lock:
            arm_state = self.arm_state
        if self.robot is None:
            raise RuntimeError("no robot_pose")
        if arm_state is None:
            raise RuntimeError("no arm_state: is robot_node running?")
        target_xy = object_in_base(self.robot, self.place_xy(goal))
        if not in_reach(target_xy, s["shoulder"], s["min_reach"], s["max_reach"]):
            raise RuntimeError(f"target at {target_xy[0]:.2f}, {target_xy[1]:.2f} m (base frame) is out of reach")
        if goal.mode == "pick":
            phases = pick_sequence(with_grasp_offset(target_xy, s["shoulder"], s.get("grasp_offset", (0, 0))), s)
        else:
            phases = place_sequence(target_xy, s)
        if self.ik is not None:
            bad = unreachable(phases, self.ik, arm_state[:5])
            if bad:
                names = ", ".join(f"{p} ({1000 * e:.0f} mm)" for p, e in bad)
                raise RuntimeError(f"the arm cannot reach: {names}; the base should stop closer")

        for phase, (x, y, z), gripper in phases:
            goal_handle.publish_feedback(PickPlace.Feedback(phase=phase))
            changed = gripper != self.gripper
            if changed:
                self.set_gripper(gripper)
            self.move_to(phase, x, y, z)
            if changed:                                   # the joints may already be there: let the jaws move
                time.sleep(s.get("gripper_settle", 0.5))
        if goal.mode == "pick":
            with self.lock:
                reading = self.arm_state[5]
            if not holding(reading, s):
                raise RuntimeError(f"missed {goal.object_name}: gripper closed to {reading:.1f} %")

    def move_to(self, phase, x, y, z):
        """Send one fingertip target and wait until the arm is there (or phase_timeout)."""
        s = self.settings
        sent = time.monotonic()
        target = PoseStamped()
        target.header.stamp, target.header.frame_id = self.get_clock().now().to_msg(), "base"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation.w = 1.0
        self.target_pub.publish(target)
        while True:
            time.sleep(0.05)
            with self.lock:
                command, when, state = self.last_command, self.command_time, self.arm_state
            waited = time.monotonic() - sent
            if when < sent:
                if waited > s.get("ik_reply_timeout", 1.0):
                    raise RuntimeError(f"{phase}: the IK node sent no command (target refused or node not running)")
                continue
            if arm_reached(state, command, s.get("joint_tolerance", 3.0)):
                return
            if waited > s["settle_time"]:
                # e.g. the fingers touch the floor or the block: carry on, the gripper check decides
                self.get_logger().warn(f"{phase}: arm not within {s.get('joint_tolerance', 3.0)} deg "
                                       f"after {s['settle_time']} s, going on")
                return

    def set_gripper(self, value):
        if not self.ik_params.wait_for_services(timeout_sec=2.0):
            raise RuntimeError("the Assignment 03 inverse_kinematics node is not running")
        future = self.ik_params.set_parameters([Parameter("gripper", value=float(value))])
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > 2.0:
                raise RuntimeError("setting the gripper timed out")
            time.sleep(0.02)
        self.gripper = value


def main():
    rclpy.init()
    node = PickPlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
