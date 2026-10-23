"""KeyboardTyper node: presses a key when it is told a letter.

Subscribes:
    key           std_msgs/String: one letter out of KEY_POSITIONS, for example "a"
    arm_state     std_msgs/Float64MultiArray: where the arm is now
Publishes:
    arm_commands  std_msgs/Float64MultiArray, 6 values: 5 joint angles (deg) and the gripper (%)
Parameters:
    move_time     seconds for each of the three moves (default MOVE_TIME). Can be changed while
                  the node runs: ros2 param set /keyboard_typer move_time 0.5

For each letter the fingertip moves to a point above the key, down onto it, and back up.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from arm_control.kinematics import inverse_kinematics

# Where the top of each key is, in the robot's base frame (metres). These values are for the
# SIMULATED keyboard. For a real keyboard, measure your own and replace them.
KEY_POSITIONS = {
    "a": [0.300, 0.070, -0.012],
    "s": [0.300, 0.000, -0.012],
    "d": [0.300, -0.070, -0.012],
}
HOVER_HEIGHT = 0.04   # metres above the key before and after pressing
PRESS_DEPTH = 0.008   # metres below the key's top to aim for when pressing
MOVE_TIME = 1.5       # seconds for each of the three moves
RATE_HZ = 20.0
FINGER_DOWN = [0.0, 0.0, -1.0]
MAX_ERROR = 0.005     # metres. A key whose targets cannot be reached this closely is not typed.
GRIPPER_CLOSED = 0.0


class KeyboardTyper(Node):
    def __init__(self):
        super().__init__("keyboard_typer")
        self.arm_state = None
        self.waypoints = []      # joint-angle targets still to visit, each a list of 6 numbers
        self.start = None        # where the current move started
        self.steps_done = 0
        # How long each of the three moves takes, in seconds. A parameter, so that the speed can be
        # changed without editing the code: --ros-args -p move_time:=0.5
        self.declare_parameter("move_time", MOVE_TIME)
        self.publisher = self.create_publisher(Float64MultiArray, "arm_commands", 10)
        self.create_subscription(String, "key", self.on_key, 10)
        self.create_subscription(Float64MultiArray, "arm_state", self.on_arm_state, 10)
        self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info(f"KeyboardTyper started. Keys: {sorted(KEY_POSITIONS)}")

    def on_arm_state(self, msg):
        self.arm_state = list(msg.data)

    def on_key(self, msg):
        letter = msg.data.strip().lower()
        if letter not in KEY_POSITIONS:
            self.get_logger().warn(f"Unknown key {letter!r}. Known keys: {sorted(KEY_POSITIONS)}")
            return
        if self.waypoints:
            self.get_logger().warn("Still busy with the previous key. Ignored.")
            return
        if self.arm_state is None:
            self.get_logger().warn("No arm_state received yet.")
            return
        key = np.array(KEY_POSITIONS[letter])
        hover = key + [0.0, 0.0, HOVER_HEIGHT]
        press = key - [0.0, 0.0, PRESS_DEPTH]
        waypoints = []
        angles = self.arm_state[:5]
        for target in (hover, press, hover):
            # Start each solve from the previous answer, so the three arm poses stay close together.
            angles, error = inverse_kinematics(target, FINGER_DOWN, angles)
            if error > MAX_ERROR:
                self.get_logger().warn(
                    f"Key {letter!r} is out of reach (error {error * 1000:.0f} mm). Not moving.")
                return
            waypoints.append([float(a) for a in angles] + [GRIPPER_CLOSED])
        self.waypoints = waypoints
        self.start = list(self.arm_state)
        self.steps_done = 0
        self.get_logger().info(f"Typing {letter!r}")

    def on_timer(self):
        if not self.waypoints:
            return
        steps_per_move = max(1, int(self.get_parameter("move_time").value * RATE_HZ))
        self.steps_done += 1
        fraction = min(1.0, self.steps_done / steps_per_move)
        start, target = np.array(self.start), np.array(self.waypoints[0])
        self.publisher.publish(Float64MultiArray(data=(start + (target - start) * fraction).tolist()))
        if fraction >= 1.0:
            self.start = self.waypoints.pop(0)
            self.steps_done = 0


def main():
    rclpy.init()
    node = KeyboardTyper()
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
