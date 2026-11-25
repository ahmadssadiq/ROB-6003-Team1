"""Publishes the wall-marker map in the aruco_map frame (the Assignment 04 map_node always uses "map").

Publishes:
    map_markers   aruco_interfaces/ArucoMarkerArray in the aruco_map frame, latched, once a second
    TF (static)   aruco_map -> marker_<id>
Parameters:
    map_file      the marker map (Assignment 04 classroom_map.yaml)

Reuses load_marker_map() and the message helpers from the Assignment 04 package unchanged.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import StaticTransformBroadcaster

from aruco_interfaces.msg import ArucoMarker, ArucoMarkerArray
from aruco_localization.map_node import load_marker_map
from aruco_localization.transforms import pose_msg_from_matrix, transform_msg_from_matrix

FRAME = "aruco_map"


class ArucoMapPublisher(Node):
    def __init__(self):
        super().__init__("aruco_map_publisher")
        self.declare_parameter("map_file", "")
        path = self.get_parameter("map_file").value
        _, self.markers = load_marker_map(path)
        self.pub = self.create_publisher(ArucoMarkerArray, "map_markers",
                                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transforms = []
        for marker_id, T in self.markers.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = FRAME, f"marker_{marker_id}"
            t.transform = transform_msg_from_matrix(T)
            transforms.append(t)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(transforms)
        self.publish()
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f"{len(self.markers)} wall markers from {path} in frame {FRAME}")

    def publish(self):
        msg = ArucoMarkerArray()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), FRAME
        msg.markers = [ArucoMarker(id=i, pose=pose_msg_from_matrix(T)) for i, T in self.markers.items()]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ArucoMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
