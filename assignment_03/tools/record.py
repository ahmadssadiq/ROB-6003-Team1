"""Record the arm's topics to CSV files, for the report's plots.

Usage (in a prepared terminal, while the robot or the simulator is running):
    python ~/FOR26/ros_ws/tools/record.py typing_run_1

Writes one CSV per topic into ~/FOR26/ros_ws/data/typing_run_1/. Stop it with Ctrl+C.
Every row starts with the time in seconds since the recording began.
"""
import csv
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
# topic name -> (message type, column names, function that turns a message into a row)
TOPICS = {
    "arm_state": (Float64MultiArray, JOINTS, lambda m: list(m.data)),
    "arm_commands": (Float64MultiArray, JOINTS, lambda m: list(m.data)),
    "fk_pose": (PoseStamped, ["x", "y", "z"],
                lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z]),
    "ik_target": (PoseStamped, ["x", "y", "z"],
                  lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z]),
    "true_fingertip": (PointStamped, ["x", "y", "z"], lambda m: [m.point.x, m.point.y, m.point.z]),
    "key_pressed": (String, ["key"], lambda m: [m.data]),
}


class Recorder(Node):
    def __init__(self, folder):
        super().__init__("recorder")
        folder.mkdir(parents=True, exist_ok=True)
        self.start = time.monotonic()
        self.files, self.writers, self.counts = {}, {}, {}
        for topic, (msg_type, columns, to_row) in TOPICS.items():
            self.files[topic] = open(folder / f"{topic}.csv", "w", newline="")
            self.writers[topic] = csv.writer(self.files[topic])
            self.writers[topic].writerow(["t"] + columns)
            self.counts[topic] = 0
            self.create_subscription(msg_type, topic, self.make_callback(topic, to_row), 50)
        self.get_logger().info(f"Recording into {folder}. Stop with Ctrl+C.")

    def make_callback(self, topic, to_row):
        def callback(msg):
            self.writers[topic].writerow([round(time.monotonic() - self.start, 4)] + to_row(msg))
            self.counts[topic] += 1
        return callback

    def close(self):
        for f in self.files.values():
            f.close()
        print("Messages recorded:", {t: n for t, n in self.counts.items() if n})


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else time.strftime("run_%H%M%S")
    folder = Path(__file__).resolve().parent.parent / "data" / name
    rclpy.init()
    node = Recorder(folder)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
