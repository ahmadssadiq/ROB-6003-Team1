"""Measure T_map_aruco once at setup: the robot's pose seen by the overhead camera (map) and by the particle
filter (aruco_map) at a few spots, fitted together (pose_check.fit_T_map_aruco).

Run with the robot stack and the overhead camera running, the overhead camera calibrated:
    ros2 run roomrunner_localization measure_t_map_aruco --output <source tree>/roomrunner_bringup/config/T_map_aruco.yaml

At each spot: drive there (teleop), let the filter settle (it should see wall markers), press Enter. The
tool asks the overhead camera for `frames` pictures (refresh_map) and averages them, and averages the filter's
pose over the same time. Use 4 to 6 spots spread over the room, not in a line. Type q when done: it prints
each spot's error and writes the file only if you confirm.
"""
import argparse
import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger

from roomrunner_localization.map_aruco_tf import yaw_of
from roomrunner_localization.pose_check import T_to_yaml, average_pose, fit_T_map_aruco

HEADER = """# Fixed transform from the ArUco room map (Assignment 04, origin below wall marker 2) to the planning
# map (overhead camera). p_map = T_map_aruco * p_aruco. A planar transform: translation and yaw.
# Measured with measure_t_map_aruco ({date}): {spots} spots, worst position error {worst:.3f} m.
"""


def pose_of(msg):
    p = msg.pose.pose if isinstance(msg, PoseWithCovarianceStamped) else msg.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, help="T_map_aruco.yaml to write")
    parser.add_argument("--frames", type=int, default=3, help="overhead pictures per spot")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("measure_t_map_aruco")
    got = {"overhead": [], "pf": []}
    node.create_subscription(PoseStamped, "overhead_robot", lambda m: got["overhead"].append(pose_of(m)), 10)
    node.create_subscription(PoseWithCovarianceStamped, "pose_estimate", lambda m: got["pf"].append(pose_of(m)), 10)
    refresh = node.create_client(Trigger, "refresh_map")
    if not refresh.wait_for_service(timeout_sec=5.0):
        raise SystemExit("refresh_map not found: start the overhead camera (overhead.launch.py) first.")

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    map_poses, aruco_poses = [], []
    while True:
        answer = input(f"Spot {len(map_poses) + 1}: drive there, stop, press Enter (q to finish) ").strip().lower()
        if answer == "q":
            break
        spin(0.5)
        got["overhead"].clear()
        got["pf"].clear()
        for _ in range(args.frames):
            future = refresh.call_async(Trigger.Request())
            while not future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
            spin(0.5)
        if not got["overhead"] or not got["pf"]:
            print(f"  skipped: {len(got['overhead'])} overhead poses (robot tag visible?), "
                  f"{len(got['pf'])} filter poses (localization running?)")
            continue
        m, a = average_pose(got["overhead"]), average_pose(got["pf"])
        map_poses.append(m)
        aruco_poses.append(a)
        print(f"  overhead (map) {m[0]:.3f} {m[1]:.3f} {math.degrees(m[2]):.0f} deg | "
              f"filter (aruco_map) {a[0]:.3f} {a[1]:.3f} {math.degrees(a[2]):.0f} deg")
    node.destroy_node()
    rclpy.shutdown()

    if len(map_poses) < 3:
        raise SystemExit("Need at least 3 spots; nothing written.")
    T, errors = fit_T_map_aruco(map_poses, aruco_poses)
    result = T_to_yaml(T, errors)
    t = result["T_map_aruco"]
    print(f"\nT_map_aruco: x {t['x']:.3f} m, y {t['y']:.3f} m, yaw {math.degrees(t['yaw']):.1f} deg")
    for i, (d, h) in enumerate(errors, 1):
        print(f"  spot {i}: {100 * d:.1f} cm, {math.degrees(h):.1f} deg")
    if input(f"Write {args.output}? [y/n] ").strip().lower() != "y":
        print("Not written.")
        return
    header = HEADER.format(date=time.strftime("%Y-%m-%d"), spots=len(errors), worst=result["worst_error_m"])
    Path(args.output).write_text(header + yaml.safe_dump(result, sort_keys=False))
    print(f"Wrote {args.output}; rebuild roomrunner_bringup and restart the robot launch.")


if __name__ == "__main__":
    main()
