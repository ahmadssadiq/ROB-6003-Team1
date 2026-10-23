"""InverseKinematics node: turns a wanted fingertip position into joint angles.

Subscribes:
    ik_target     geometry_msgs/PoseStamped: where the fingertip should go, in the robot's base
                  frame. The finger is always made to point straight down.
    arm_state     std_msgs/Float64MultiArray: where the arm is now (the starting point of the solve)
Publishes:
    arm_commands  std_msgs/Float64MultiArray, 6 values: 5 joint angles (deg) and the gripper (%)
Parameters:
    gripper       the gripper value to send along, in percent (default 0.0 = closed)
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from arm_control.kinematics import inverse_kinematics

FINGER_DOWN = [0.0, 0.0, -1.0]
MAX_ERROR = 0.005   # metres. A solution further than this from the target is not sent.


class InverseKinematics(Node):
    def __init__(self):
        super().__init__("inverse_kinematics")
        self.declare_parameter("gripper", 0.0)
        self.arm_state = None
        self.publisher = self.create_publisher(Float64MultiArray, "arm_commands", 10)
        self.create_subscription(PoseStamped, "ik_target", self.on_target, 10)
        self.create_subscription(Float64MultiArray, "arm_state", self.on_arm_state, 10)
        self.get_logger().info("InverseKinematics started.")

    def on_arm_state(self, msg):
        self.arm_state = list(msg.data)

    def on_target(self, msg):
        if self.arm_state is None:
            self.get_logger().warn("No arm_state received yet, cannot solve.")
            return
        target = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        angles, error = inverse_kinematics(target, FINGER_DOWN, self.arm_state)
        if error > MAX_ERROR:
            self.get_logger().warn(
                f"Target {target} is out of reach (error {error * 1000:.0f} mm). Not moving.")
            return
        command = Float64MultiArray()
        command.data = [float(a) for a in angles] + [float(self.get_parameter("gripper").value)]
        self.publisher.publish(command)
        self.get_logger().info(
            f"Target {target} -> joint angles {[round(float(a), 1) for a in angles]} deg")


def main():
    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
