"""Stand-in for the robot and its localization: a point robot that drives exactly as commanded.

Subscribes:
    base_commands  std_msgs/Float64MultiArray [forward m/s, left m/s, turn deg/s], the same topic and units
                   as the real robot_node (course code)
Publishes:
    robot_pose     geometry_msgs/PoseStamped in the map frame, 20 times a second
    TF             map -> base
Parameters:
    start          [x, y, yaw] in the map frame (m, m, rad)

With the real robot, robot_pose comes from the particle filter (Assignment 04) through map_aruco_tf.
The wheels stop if no command arrives for 0.5 s, as on the real robot.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

RATE_HZ = 20.0
COMMAND_TIMEOUT = 0.5   # s, as BASE_COMMAND_TIMEOUT in robot_node


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        self.declare_parameter("start", [0.0, 0.0, 0.0])
        self.x, self.y, self.yaw = self.get_parameter("start").value
        self.velocity = [0.0, 0.0, 0.0]     # forward m/s, left m/s, turn rad/s
        self.last_command = None

        self.pose_pub = self.create_publisher(PoseStamped, "robot_pose", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, "base_commands", self.on_command, 10)
        self.create_timer(1.0 / RATE_HZ, self.step)
        self.get_logger().info(f"Fake robot at x={self.x:.2f} y={self.y:.2f} yaw={self.yaw:.2f}")

    def on_command(self, msg):
        forward, left, turn_deg = msg.data
        self.velocity = [forward, left, math.radians(turn_deg)]   # /base_commands turns in deg/s
        self.last_command = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        if self.last_command is None or (now - self.last_command).nanoseconds / 1e9 > COMMAND_TIMEOUT:
            self.velocity = [0.0, 0.0, 0.0]
        forward, left, turn = self.velocity
        dt = 1.0 / RATE_HZ
        # The command is in the robot's own frame: rotate it into the map frame.
        self.x += (forward * math.cos(self.yaw) - left * math.sin(self.yaw)) * dt
        self.y += (forward * math.sin(self.yaw) + left * math.cos(self.yaw)) * dt
        self.yaw = math.atan2(math.sin(self.yaw + turn * dt), math.cos(self.yaw + turn * dt))

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = self.x, self.y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.pose_pub.publish(pose)

        t = TransformStamped()
        t.header = pose.header
        t.child_frame_id = "base"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation = pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
