"""Watches the particle filter against the overhead camera and re-seeds it when it has lost track.

Subscribes:
    robot_pose      geometry_msgs/PoseStamped in the map frame (particle filter, through map_aruco_tf)
    overhead_robot  geometry_msgs/PoseStamped in the map frame (robot-top tag, overhead_node, every ~10 s)
Publishes:
    initialpose         geometry_msgs/PoseWithCovarianceStamped in the aruco_map frame: the overhead pose,
                        when a reset is due (the Assignment 04 localization_node spreads its particles there)
    localization_check  std_msgs/String: the result of each check, e.g. "ok 0.04 m 2 deg" (for logs and E1)
Parameters:
    transform_file   T_map_aruco.yaml, to turn the overhead pose into the aruco_map frame
    reset            true: re-seed the filter; false: only report       (default true)
    max_distance     m (default 0.40)     max_heading_deg (default 25)    strikes (default 2)
    cooldown         s between resets (default 30)
    still_window     s the robot must have been still before an overhead pose arrives (default 2.0)

Logic: pose_check.PoseMonitor. Overhead poses are timed by when they arrive here, not by their stamp,
because the laptop's clock may differ from the mini PC's; the robot must have been still for still_window
seconds before, so the frame's capture and network delay do not matter.
"""
import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from roomrunner_localization.map_aruco_tf import yaw_of
from roomrunner_localization.pose_check import PoseHistory, PoseMonitor
from roomrunner_localization.transform2d import invert, matrix, transform_pose


class PoseMonitorNode(Node):
    def __init__(self):
        super().__init__("pose_monitor")
        for name, default in [("transform_file", ""), ("reset", True), ("max_distance", 0.40),
                              ("max_heading_deg", 25.0), ("strikes", 2), ("cooldown", 30.0), ("still_window", 2.0)]:
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value
        with open(p("transform_file")) as f:
            t = yaml.safe_load(f)["T_map_aruco"]
        self.T_aruco_map = invert(matrix(t["x"], t["y"], t["yaw"]))
        self.monitor = PoseMonitor(p("max_distance"), math.radians(p("max_heading_deg")), p("strikes"), p("cooldown"))
        self.history = PoseHistory(seconds=max(5.0, 2 * p("still_window")))
        self.reset, self.window = p("reset"), p("still_window")

        self.create_subscription(PoseStamped, "robot_pose", self.on_pf, 20)
        self.create_subscription(PoseStamped, "overhead_robot", self.on_overhead, 10)
        self.reset_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.check_pub = self.create_publisher(String, "localization_check", 10)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pf(self, msg):
        self.history.add(self.now(), (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation)))

    def on_overhead(self, msg):
        t = self.now()
        overhead = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))
        pf = self.history.nearest(t)
        action = self.monitor.check(t, pf, overhead, self.history.still(t, self.window))
        detail = action
        if pf is not None and action != "skip":
            heading = math.degrees(math.atan2(math.sin(pf[2] - overhead[2]), math.cos(pf[2] - overhead[2])))
            detail += f" {math.hypot(pf[0] - overhead[0], pf[1] - overhead[1]):.2f} m {heading:.0f} deg"
        if action == "reset" and self.reset:
            self.publish_reset(overhead)
            detail += " (filter re-seeded at the overhead pose)"
        elif action == "reset":
            detail += " (reset is off)"
        self.check_pub.publish(String(data=detail))
        if action in ("disagree", "reset"):
            self.get_logger().warn(f"localization: {detail}")

    def publish_reset(self, overhead_map):
        x, y, yaw = transform_pose(self.T_aruco_map, overhead_map)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), "aruco_map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        covariance = np.zeros((6, 6))
        covariance[0, 0] = covariance[1, 1] = 0.03 ** 2      # overhead tag: a few cm, a few degrees
        covariance[5, 5] = math.radians(5) ** 2
        msg.pose.covariance = covariance.ravel().tolist()
        self.reset_pub.publish(msg)


def main():
    rclpy.init()
    node = PoseMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
