"""Try one pick (and optionally put it back) at a spot in front of the robot, without a mission. For measuring
grasp_z, grasp_offset and the gripper values in manipulation.yaml on the real arm.

    ros2 launch roomrunner_bringup robot.launch.py port:=... enable_motors:=true arm:=real
    ros2 run roomrunner_manipulation try_pick --x 0.25 --y 0.0 [--place]

--x, --y: the block's position in the base frame (m, x forward, y left): put the block there with a ruler.
pick_place_server reads manipulation.yaml again for every goal, so edit the file (the one it was started
with: settings_file) and run try_pick again. Prints each phase and the result ("missed ..." when the
gripper closed on nothing).
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

from roomrunner_interfaces.action import PickPlace


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--x", type=float, required=True, help="block x in the base frame (m)")
    parser.add_argument("--y", type=float, default=0.0, help="block y in the base frame (m)")
    parser.add_argument("--place", action="store_true", help="after the pick, put the block back down there")
    parser.add_argument("--name", default="test_block")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("try_pick")
    robot = {}
    node.create_subscription(PoseStamped, "robot_pose", lambda m: robot.update(m=m), 10)
    client = ActionClient(node, PickPlace, "pick_place")
    if not client.wait_for_server(timeout_sec=5.0):
        raise SystemExit("pick_place_server is not running.")
    while "m" not in robot:
        rclpy.spin_once(node, timeout_sec=0.1)
    m = robot["m"]
    rx, ry, yaw = m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)

    def run(mode):
        goal = PickPlace.Goal(mode=mode, object_name=args.name)
        goal.target.header.frame_id = "map"
        # the spot in the map frame, from the base frame (the server turns it back)
        goal.target.pose.position.x = rx + math.cos(yaw) * args.x - math.sin(yaw) * args.y
        goal.target.pose.position.y = ry + math.sin(yaw) * args.x + math.cos(yaw) * args.y
        goal.target.pose.orientation.w = 1.0
        sent = client.send_goal_async(goal, feedback_callback=lambda f: print(f"  {mode}: {f.feedback.phase}"))
        rclpy.spin_until_future_complete(node, sent)
        done = sent.result().get_result_async()
        rclpy.spin_until_future_complete(node, done)
        result = done.result().result
        print(f"{mode}: {'OK' if result.success else 'FAILED'} {result.message}")
        return result.success

    if run("pick") and args.place:
        run("place")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
