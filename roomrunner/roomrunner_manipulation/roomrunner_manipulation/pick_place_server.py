"""Pick and place action server. STUB: reports each phase and succeeds, without moving the arm.

Action server:
    pick_place     roomrunner_interfaces/PickPlace: mode ("pick" or "place"), object name, target pose (map)
Parameters:
    phase_time     s spent in each phase (default 0.5)

The real version (owner: Sarvanagupta) moves the arm through the Assignment 03 EEControl node: it turns
the target from the map frame into the base frame, publishes /ik_target (PoseStamped, base frame) for the
approach, grasp and lift poses, and opens and closes the gripper through /arm_commands.
"""
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from roomrunner_interfaces.action import PickPlace

PHASES = {"pick": ["approach", "grasp", "lift"], "place": ["carry", "release", "retreat"]}


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__("pick_place_server")
        self.declare_parameter("phase_time", 0.5)
        self.server = ActionServer(self, PickPlace, "pick_place", self.execute)
        self.get_logger().info("Pick and place server ready (stub: the arm does not move).")

    def execute(self, goal_handle):
        goal = goal_handle.request
        result = PickPlace.Result()
        if goal.mode not in PHASES:
            goal_handle.abort()
            result.message = f"unknown mode {goal.mode!r}"
            return result
        for phase in PHASES[goal.mode]:
            goal_handle.publish_feedback(PickPlace.Feedback(phase=phase))
            self.get_logger().info(f"{goal.mode} {goal.object_name}: {phase}")
            time.sleep(self.get_parameter("phase_time").value)
        goal_handle.succeed()
        result.success = True
        result.message = f"{goal.mode} {goal.object_name} done (stub)"
        return result


def main():
    rclpy.init()
    node = PickPlaceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
