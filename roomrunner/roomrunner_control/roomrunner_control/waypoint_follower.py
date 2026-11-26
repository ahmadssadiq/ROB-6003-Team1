"""Waypoint follower: drives the LeKiwi base along a path and stops at its end, aligned with its heading,
then (optionally) corrects the stop with the overhead camera.

Action server:
    navigate_to    roomrunner_interfaces/NavigateTo: path (map frame) and tolerances
Subscribes:
    robot_pose     geometry_msgs/PoseStamped in the map frame (20 Hz, map_aruco_tf or fake_robot)
    overhead_robot geometry_msgs/PoseStamped in the map frame: the robot-top tag (overhead_node), for alignment
Calls:
    refresh_map    std_srvs/Trigger: asks the overhead camera for a picture now (alignment)
Publishes:
    base_commands  std_msgs/Float64MultiArray [forward m/s, left m/s, turn deg/s] (robot_node / sim_node units)
Parameters:
    max_speed      m/s (default 0.15)            max_turn        rad/s (default 0.5)
    gain           1/s, on the position error (default 1.5)
    yaw_gain       1/s, on the heading error (default 1.5)
    lookahead      m: move on to the next waypoint once this close (default 0.15)
    min_speed      m/s: smallest drive command, against the motors' deadband (default 0.03)
    max_accel      m/s^2 (default 0.3)           max_turn_accel  rad/s^2 (default 1.5)
    pose_timeout   s: stop and wait when robot_pose is older than this (default 0.5)
    timeout        s: give up after this long (default 120)
    align_with_overhead  true: after stopping, re-drive by the overhead camera's error (default true)
    align_attempts       corrections at most (default 2)
    align_timeout        s to wait for an overhead pose; none: alignment is skipped (default 3)

The base is omnidirectional, so it drives straight at the next waypoint in any direction while it turns
towards the goal heading (controller.py). Tune the gains offline first: python3 -m roomrunner_control.sim_base

Alignment: once stopped, the filter's pose and the overhead camera's view of the robot-top tag are compared.
Their difference is the filter's local error, so the follower drives to the goal shifted by it
(controller.overhead_correction) and checks again. The result reports the overhead camera's final error.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from roomrunner_control.controller import (at_goal, limit_acceleration, next_index, overhead_correction,
                                           remaining_distance, velocity_command, with_min_speed, wrap)
from roomrunner_interfaces.action import NavigateTo

RATE_HZ = 20.0
SETTLE_TIME = 0.5   # s the robot must stay inside the tolerances before it counts as stopped
PARAMETERS = (("max_speed", 0.15), ("max_turn", 0.5), ("gain", 1.5), ("yaw_gain", 1.5), ("lookahead", 0.15),
              ("min_speed", 0.03), ("max_accel", 0.3), ("max_turn_accel", 1.5), ("pose_timeout", 0.5),
              ("timeout", 120.0), ("align_with_overhead", True), ("align_attempts", 2), ("align_timeout", 3.0))


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)
        self.pose = self.overhead = None
        self.pose_time = self.overhead_time = 0.0
        self.command = (0.0, 0.0, 0.0)
        self.lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self.create_subscription(PoseStamped, "robot_pose", self.on_pose, 10, callback_group=group)
        self.create_subscription(PoseStamped, "overhead_robot", self.on_overhead, 10, callback_group=group)
        self.refresh = self.create_client(Trigger, "refresh_map", callback_group=group)
        self.command_pub = self.create_publisher(Float64MultiArray, "base_commands", 10)
        self.server = ActionServer(self, NavigateTo, "navigate_to", self.execute, callback_group=group)
        self.get_logger().info("Waypoint follower ready.")

    @staticmethod
    def pose_of(msg):
        return msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose)

    def on_pose(self, msg):
        with self.lock:
            self.pose, self.pose_time = self.pose_of(msg), time.monotonic()

    def on_overhead(self, msg):
        with self.lock:
            self.overhead, self.overhead_time = self.pose_of(msg), time.monotonic()

    def send(self, command, p=None):
        """Publish a command, acceleration-limited when p (the parameters) is given."""
        if p is not None:
            command = limit_acceleration(self.command, command, p["max_accel"], p["max_turn_accel"], 1.0 / RATE_HZ)
        self.command = command
        forward, left, turn = command
        # /base_commands takes the turn rate in deg/s.
        self.command_pub.publish(Float64MultiArray(data=[float(forward), float(left), math.degrees(turn)]))

    def stop(self):
        self.send((0.0, 0.0, 0.0))

    def drive(self, goal_handle, points, goal_yaw, tolerances, p, deadline):
        """Follow points to goal_yaw and stop there. Returns "arrived", "canceled" or "timed out"."""
        index, settled_since = 0, None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return "canceled"
            if time.monotonic() > deadline:
                return "timed out"
            with self.lock:
                pose, age = self.pose, time.monotonic() - self.pose_time
            if pose is None or age > p["pose_timeout"]:
                self.send((0.0, 0.0, 0.0), p)          # no fresh pose: slow down and wait for one
                time.sleep(1.0 / RATE_HZ)
                continue
            index = next_index(points, index, pose[:2], p["lookahead"])
            if index == len(points) - 1 and at_goal(pose, points[-1], goal_yaw, *tolerances):
                self.send((0.0, 0.0, 0.0), p)
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= SETTLE_TIME:
                    self.stop()
                    return "arrived"
            else:
                settled_since = None
                command = velocity_command(pose, points[index], goal_yaw, p["max_speed"], p["gain"],
                                           p["max_turn"], p["yaw_gain"])
                self.send(with_min_speed(command, p["min_speed"]), p)
            feedback = NavigateTo.Feedback()
            feedback.distance_remaining = remaining_distance(pose[:2], points, index)
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0 / RATE_HZ)
        return "canceled"

    def overhead_pose(self, timeout):
        """A fresh overhead pose of the robot (asks the camera for a picture), or None within timeout s."""
        if not self.refresh.service_is_ready():
            return None
        asked = time.monotonic()
        self.refresh.call_async(Trigger.Request())
        while time.monotonic() - asked < timeout:
            with self.lock:
                if self.overhead_time > asked:
                    return self.overhead
            time.sleep(0.05)
        return None

    def align(self, goal_handle, goal, goal_yaw, tolerances, p, deadline, result):
        """Correct the stop with the overhead camera. Returns (status, note for the result message)."""
        for attempt in range(p["align_attempts"] + 1):
            overhead = self.overhead_pose(p["align_timeout"])
            with self.lock:
                pf = self.pose
            if overhead is None or pf is None:
                return "arrived", ", no overhead pose: not checked with the camera"
            target, target_yaw, error = overhead_correction(goal, goal_yaw, pf, overhead)
            yaw_error = wrap(goal_yaw - overhead[2])
            result.final_position_error, result.final_yaw_error = error, yaw_error
            if error < tolerances[0] and abs(yaw_error) < tolerances[1]:
                return "arrived", f", overhead camera error {100 * error:.1f} cm"
            if attempt == p["align_attempts"]:
                return "arrived", f", overhead camera error still {100 * error:.1f} cm"
            self.get_logger().info(f"Overhead camera: {100 * error:.1f} cm off, correcting")
            status = self.drive(goal_handle, [target], target_yaw, tolerances, p, deadline)
            if status != "arrived":
                return status, ""
        return "arrived", ""

    def execute(self, goal_handle):
        goal = goal_handle.request
        points = [(q.pose.position.x, q.pose.position.y) for q in goal.path.poses]
        result = NavigateTo.Result()
        if not points:
            goal_handle.abort()
            result.message = "empty path"
            return result
        goal_yaw = yaw_of(goal.path.poses[-1].pose)
        p = {name: self.get_parameter(name).value for name, _ in PARAMETERS}
        tolerances = (goal.position_tolerance, goal.yaw_tolerance)
        deadline = time.monotonic() + p["timeout"]

        status = self.drive(goal_handle, points, goal_yaw, tolerances, p, deadline)
        with self.lock:
            pose = self.pose
        if status == "arrived":
            result.final_position_error = math.dist(pose[:2], points[-1])      # the filter's view
            result.final_yaw_error = wrap(goal_yaw - pose[2])
        note = ""
        if status == "arrived" and p["align_with_overhead"]:
            status, note = self.align(goal_handle, points[-1], goal_yaw, tolerances, p, deadline, result)

        self.stop()
        if status == "canceled":
            goal_handle.canceled()
            result.message = "canceled"
        elif status == "timed out":
            goal_handle.abort()
            result.message = "timed out"
        else:
            goal_handle.succeed()
            result.success = True
            result.message = "aligned and stopped" + note
        return result


def main():
    rclpy.init()
    node = WaypointFollower()
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
