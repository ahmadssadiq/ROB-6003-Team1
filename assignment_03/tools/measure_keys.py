"""Measure the P, O and L keys with the robot's own forward kinematics and update ros_ws/ee.

Run it while the real robot is running in READ-ONLY mode (motors off), so you can move the arm
by hand:
    ros2 launch arm_control typing_real.launch.py port:=/dev/serial/by-id/usb-...      (terminal 1)
    python ~/FOR26/ros_ws/tools/measure_keys.py                                        (terminal 2)

For each key it asks you to hold the fingertip on the key, pressed down, finger pointing straight
down, and press Enter. It then reads /fk_pose. Home is set 5 cm above the middle of the three
keys (lower if the arm cannot reach that high with the finger pointing down).
"""
import re
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped

from arm_control.kinematics import inverse_kinematics

EE_FILE = Path(__file__).resolve().parent.parent / "ee"
KEYS = ("p", "o", "l")
HOME_HEIGHT = 0.05      # metres above the middle of the keys
MAX_ERROR = 0.005       # the same limit the IK node uses


def main():
    rclpy.init()
    node = rclpy.create_node("measure_keys")
    latest = {}
    node.create_subscription(PoseStamped, "fk_pose", lambda m: latest.update(
        pose=np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]), time=time.monotonic()), 10)

    print("Waiting for /fk_pose ...")
    while "pose" not in latest:
        rclpy.spin_once(node, timeout_sec=0.2)

    measured = {}
    for key in KEYS:
        input(f"\nHold the fingertip on the {key.upper()} key, pressed down, finger straight down. "
              f"Then press Enter.")
        # Average half a second of readings, so a small wobble does not matter.
        samples, end = [], time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            samples.append(latest["pose"])
        measured[key] = np.mean(samples, axis=0)
        x, y, z = measured[key]
        print(f"  {key.upper()}: x={x:.3f} y={y:.3f} z={z:.3f}")

    centre = np.mean([measured[k] for k in KEYS], axis=0)
    spacing = np.linalg.norm(measured["p"] - measured["o"]) * 1000
    print(f"\nP to O is {spacing:.0f} mm apart (a keyboard is about 19 mm).")

    # Home: as high as possible up to HOME_HEIGHT, with the finger still pointing straight down.
    home = None
    for height in np.arange(HOME_HEIGHT, 0.019, -0.005):
        candidate = centre + [0.0, 0.0, height]
        _, error = inverse_kinematics(candidate, [0, 0, -1], [10, 45, -20, 85, 0])
        if error < MAX_ERROR:
            home = candidate
            break
    if home is None:
        print("The arm cannot reach 2 cm above these keys with the finger pointing down. ee not changed.")
        return
    print(f"Home: {(home - centre)[2] * 100:.1f} cm above the keys: "
          f"x={home[0]:.3f} y={home[1]:.3f} z={home[2]:.3f}")

    for key in KEYS:
        _, error = inverse_kinematics(measured[key], [0, 0, -1], [10, 45, -20, 85, 0])
        if error > MAX_ERROR:
            print(f"Warning: the IK node will refuse {key.upper()} (out of reach by {error * 1000:.0f} mm).")

    text = EE_FILE.read_text()
    for name, point in {"home": home, **measured}.items():
        new = f'xyz="{point[0]:.3f} {point[1]:.3f} {point[2]:.3f}"'
        text = re.sub(rf'(\b{name}\)\s+)xyz="[^"]*"', rf"\g<1>{new}", text)
    EE_FILE.write_text(text)
    print(f"\nUpdated {EE_FILE}:")
    for line in text.splitlines():
        if 'xyz="' in line:
            print("   ", line.strip())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
