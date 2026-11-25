"""Pick and place action server, on top of the Assignment 03 EEControl node (inverse_kinematics).

Action server:
    pick_place     roomrunner_interfaces/PickPlace: mode ("pick" or "place"), object name, target pose (map)
Subscribes:
    robot_pose     geometry_msgs/PoseStamped (map): where the base is, to turn the target into the base frame
Publishes:
    ik_target      geometry_msgs/PoseStamped (base frame): fingertip targets for the Assignment 03 IK node
Parameters:
    mode           "stub" (default: report phases, move nothing) or "real"
    settings_file  manipulation.yaml (heights, gripper values, reach, settle time)
    phase_time     s per phase in stub mode (default 0.5)

In real mode every arm command goes through the Assignment 03 IK node, the gripper too: before a phase
that changes the gripper, this node sets the IK node's own "gripper" parameter, then sends the target.
So only one node ever publishes /arm_commands.
"""
import math
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

from roomrunner_interfaces.action import PickPlace
from roomrunner_manipulation.grasp import in_reach, object_in_base, pick_sequence, place_sequence

STUB_PHASES = {"pick": ["approach", "descend", "grasp", "lift"], "place": ["carry", "lower", "release", "retreat"]}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__("pick_place_server")
        self.declare_parameter("mode", "stub")
        self.declare_parameter("settings_file", "")
        self.declare_parameter("phase_time", 0.5)
        self.mode = self.get_parameter("mode").value
        self.settings = {}
        if self.get_parameter("settings_file").value:
            with open(self.get_parameter("settings_file").value) as f:
                self.settings = yaml.safe_load(f)
        group = ReentrantCallbackGroup()
        self.robot = None
        self.create_subscription(PoseStamped, "robot_pose", self.on_pose, 10, callback_group=group)
        self.target_pub = self.create_publisher(PoseStamped, "ik_target", 10)
        self.ik_params = AsyncParameterClient(self, "inverse_kinematics", callback_group=group)
        self.gripper = None
        self.server = ActionServer(self, PickPlace, "pick_place", self.execute, callback_group=group)
        self.get_logger().info(f"Pick and place server ready ({self.mode} mode).")

    def on_pose(self, msg):
        self.robot = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))

    def execute(self, goal_handle):
        goal = goal_handle.request
        result = PickPlace.Result()
        if goal.mode not in STUB_PHASES:
            goal_handle.abort()
            result.message = f"unknown mode {goal.mode!r}"
            return result
        try:
            if self.mode == "stub":
                for phase in STUB_PHASES[goal.mode]:
                    goal_handle.publish_feedback(PickPlace.Feedback(phase=phase))
                    time.sleep(self.get_parameter("phase_time").value)
            else:
                self.run_real(goal_handle, goal)
        except RuntimeError as error:
            goal_handle.abort()
            result.message = str(error)
            self.get_logger().warn(f"{goal.mode} {goal.object_name} failed: {error}")
            return result
        goal_handle.succeed()
        result.success = True
        result.message = f"{goal.mode} {goal.object_name} done" + (" (stub)" if self.mode == "stub" else "")
        return result

    def run_real(self, goal_handle, goal):
        s = self.settings
        if self.robot is None:
            raise RuntimeError("no robot_pose")
        target_xy = object_in_base(self.robot, (goal.target.pose.position.x, goal.target.pose.position.y))
        if not in_reach(target_xy, s["shoulder"], s["min_reach"], s["max_reach"]):
            raise RuntimeError(f"target at {target_xy[0]:.2f}, {target_xy[1]:.2f} m (base frame) is out of reach")
        phases = pick_sequence(target_xy, s) if goal.mode == "pick" else place_sequence(target_xy, s)
        for phase, (x, y, z), gripper in phases:
            goal_handle.publish_feedback(PickPlace.Feedback(phase=phase))
            if gripper != self.gripper:
                self.set_gripper(gripper)
            target = PoseStamped()
            target.header.stamp, target.header.frame_id = self.get_clock().now().to_msg(), "base"
            target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
            target.pose.orientation.w = 1.0
            self.target_pub.publish(target)
            time.sleep(s["settle_time"])

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
