"""Log poses during a run to CSV, for E1 (run on the mini PC while the robot drives).

    python3 experiments/log_poses.py run1      -> experiments/data/run1_poses.csv

Columns: t, source, x, y, yaw. Sources:
    odom            /odom (nav_msgs/Odometry), odometry only (odom frame; starts where the robot started)
    pf              /robot_pose (map frame), particle filter through map_aruco_tf
    overhead        /overhead_robot (map frame), the robot-top tag seen by the overhead camera
Stop with Ctrl+C. Checkpoint ground truth (tape-measured) goes into e1_checkpoints.csv by hand.
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from pathlib import Path

OUT = Path(__file__).resolve().parent / "data"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else time.strftime("run_%H%M%S")
    OUT.mkdir(exist_ok=True)
    f = open(OUT / f"{name}_poses.csv", "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["t", "source", "x", "y", "yaw"])
    start = time.monotonic()

    def log(source, pose):
        writer.writerow([f"{time.monotonic() - start:.3f}", source, f"{pose.position.x:.4f}",
                         f"{pose.position.y:.4f}", f"{yaw_of(pose.orientation):.4f}"])

    rclpy.init()
    node = rclpy.create_node("log_poses")
    node.create_subscription(Odometry, "odom", lambda m: log("odom", m.pose.pose), 10)
    node.create_subscription(PoseStamped, "robot_pose", lambda m: log("pf", m.pose), 10)
    node.create_subscription(PoseStamped, "overhead_robot", lambda m: log("overhead", m.pose), 10)
    print(f"Logging to {f.name}. Ctrl+C to stop.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
