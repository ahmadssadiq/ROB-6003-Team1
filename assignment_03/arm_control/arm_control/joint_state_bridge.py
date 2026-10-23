"""ROS 2 node that lets RViz show the arm.

RViz draws a robot from two things: a description of its parts (the URDF file) and the current
angle of every joint, on the standard topic joint_states. Our robot publishes its angles on
arm_state instead, in the course's own units. This node translates one into the other.

Subscribes:
    arm_state     std_msgs/Float64MultiArray, 6 values: 5 joint angles (deg) and the gripper (%)
Publishes:
    joint_states  sensor_msgs/JointState: the same six joints, in radians, under the joint names
                  the robot model uses

It works the same whether arm_state comes from the simulator (sim_node) or the real robot
(robot_node).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from lekiwi_sim import LeKiwiSim
from lekiwi_sim.simulator import MODEL_ARM_JOINTS


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(Float64MultiArray, "arm_state", self.on_state, 10)
        self.get_logger().info("Translating arm_state into joint_states for RViz.")

    def on_state(self, msg):
        if len(msg.data) != 6:
            self.get_logger().warn(f"arm_state needs 6 values, got {len(msg.data)}. Ignored.")
            return
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = list(MODEL_ARM_JOINTS)
        # The robot model's joints have a different zero pose and turning direction than the
        # real robot. The simulator already knows the conversion, so use the same one here.
        joint_states.position = LeKiwiSim._arm_to_model(list(msg.data)).tolist()
        self.publisher.publish(joint_states)


def main():
    rclpy.init()
    node = JointStateBridge()
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
