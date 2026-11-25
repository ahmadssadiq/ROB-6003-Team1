"""Connects the particle filter's frame (aruco_map) to the planning map (map).

Publishes:
    TF (static)    map -> aruco_map, from T_map_aruco in the config file
    robot_pose     geometry_msgs/PoseStamped in the map frame: the particle filter's estimate, transformed
Subscribes:
    pose_estimate  geometry_msgs/PoseWithCovarianceStamped in the aruco_map frame (Assignment 04 localization_node)
Parameters:
    transform_file  YAML with T_map_aruco: {x, y, yaw} (roomrunner_bringup/config/T_map_aruco.yaml)
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from roomrunner_localization.transform2d import matrix, transform_pose


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapArucoTF(Node):
    def __init__(self):
        super().__init__("map_aruco_tf")
        self.declare_parameter("transform_file", "")
        with open(self.get_parameter("transform_file").value) as f:
            config = yaml.safe_load(f)
        t = config["T_map_aruco"]
        self.T_map_aruco = matrix(t["x"], t["y"], t["yaw"])
        if not config.get("measured", False):
            self.get_logger().warn("T_map_aruco is a placeholder (measured: false); measure it before trusting poses.")

        static = TransformStamped()
        static.header.stamp = self.get_clock().now().to_msg()
        static.header.frame_id, static.child_frame_id = "map", "aruco_map"
        static.transform.translation.x, static.transform.translation.y = float(t["x"]), float(t["y"])
        static.transform.rotation.z, static.transform.rotation.w = math.sin(t["yaw"] / 2), math.cos(t["yaw"] / 2)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(static)

        self.pose_pub = self.create_publisher(PoseStamped, "robot_pose", 10)
        self.create_subscription(PoseWithCovarianceStamped, "pose_estimate", self.on_estimate, 10)
        self.get_logger().info(f"map -> aruco_map: x={t['x']} y={t['y']} yaw={t['yaw']}")

    def on_estimate(self, msg):
        p = msg.pose.pose
        x, y, yaw = transform_pose(self.T_map_aruco, (p.position.x, p.position.y, yaw_of(p.orientation)))
        out = PoseStamped()
        out.header.stamp, out.header.frame_id = msg.header.stamp, "map"
        out.pose.position.x, out.pose.position.y = x, y
        out.pose.orientation.z, out.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pose_pub.publish(out)


def main():
    rclpy.init()
    node = MapArucoTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
