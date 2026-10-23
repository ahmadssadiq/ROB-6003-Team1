"""Exercise 1: a "leader" node that publishes arm commands.

When it is finished, this node makes the simulated arm wave: it publishes a new arm command
20 times per second, in which shoulder_pan swings between -40 and +40 degrees.

Fill in the three TODOs.
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

RATE_HZ = 20.0         # how many commands to publish per second
AMPLITUDE_DEG = 40.0   # how far shoulder_pan swings to each side
PERIOD_S = 4.0         # seconds for one full swing left, right and back


class Leader(Node):
    def __init__(self):
        super().__init__("leader")
        self.start_time = time.monotonic()

        # TODO 1: create a publisher for messages of type Float64MultiArray on the topic
        #         "arm_commands", and keep it in self.publisher.
        #         Hint: self.create_publisher(message type, topic name, 10)
        self.publisher = self.create_publisher(Float64MultiArray, "arm_commands", 10)

        # TODO 2: create a timer that calls self.on_timer RATE_HZ times per second.
        #         Hint: self.create_timer(seconds between calls, function to call)
        self.create_timer(1.0 / RATE_HZ, self.on_timer)

        self.get_logger().info("Leader started.")

    def on_timer(self):
        elapsed = time.monotonic() - self.start_time
        shoulder_pan = AMPLITUDE_DEG * math.sin(2.0 * math.pi * elapsed / PERIOD_S)

        # TODO 3: create a Float64MultiArray message, fill its .data with six numbers
        #         [shoulder_pan, 0, 0, 0, 0, 50] (the last one is the gripper, in percent),
        #         and publish it with self.publisher.
        msg = Float64MultiArray()
        msg.data = [shoulder_pan, 0.0, 0.0, 0.0, 0.0, 50.0]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = Leader()
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
