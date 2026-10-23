"""Simulation experiment: how fast can the arm type before key presses are missed?

Needs sim_node running with the keyboard scene. For each move time it starts keyboard_typer with
that move time, asks it to type a, s and d, and counts which keys the simulator reports pressed.

Usage:  python ~/FOR26/ros_ws/tools/speed_test.py
Writes: ~/FOR26/ros_ws/data/sim_speed/speed_vs_success.csv
"""
import csv
import subprocess
import time
from pathlib import Path

import rclpy
from std_msgs.msg import String

MOVE_TIMES = [1.0, 0.7, 0.5, 0.4, 0.3, 0.2, 0.15, 0.1]
KEYS = ["a", "s", "d"] * 3   # each key three times per speed
OUT = Path(__file__).resolve().parent.parent / "data" / "sim_speed"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("speed_test")
    pressed = []
    node.create_subscription(String, "key_pressed", lambda m: pressed.append(m.data), 10)
    key_pub = node.create_publisher(String, "key", 10)

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    rows = []
    for move_time in MOVE_TIMES:
        typer = subprocess.Popen(["ros2", "run", "arm_control", "keyboard_typer", "--ros-args",
                                  "-p", f"move_time:={move_time}"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        while key_pub.get_subscription_count() == 0:
            wait(0.1)
        wait(2.0)   # let the typer receive arm_state
        for key in KEYS:
            pressed.clear()
            key_pub.publish(String(data=key))
            wait(3 * move_time + 1.5)
            hit = f"key_{key}" in pressed
            wrong = [p for p in pressed if p != f"key_{key}"]
            rows.append([move_time, key, int(hit), " ".join(wrong)])
            print(f"move time {move_time:.2f} s, key {key}: {'pressed' if hit else 'MISSED'}"
                  + (f", also pressed {wrong}" if wrong else ""))
        typer.terminate()
        typer.wait()
        wait(1.0)

    with open(OUT / "speed_vs_success.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["move_time", "key", "pressed", "other_keys_pressed"])
        writer.writerows(rows)
    print("wrote", OUT / "speed_vs_success.csv")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
