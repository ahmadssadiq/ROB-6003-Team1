"""Try one pick (and optionally put it back) at a spot in front of the robot, without a mission. For measuring
grasp_z, grasp_offset and the gripper values in manipulation.yaml on the real arm.

    ros2 launch roomrunner_bringup robot.launch.py port:=... enable_motors:=true arm:=real
    ros2 run roomrunner_manipulation try_pick --x 0.25 --y 0.0 [--place]

--x, --y: the block's position in the base frame (m, x forward, y left): put the block there with a ruler.
pick_place_server reads manipulation.yaml again for every goal, so edit the file (the one it was started
with: settings_file) and run try_pick again. Prints each phase and the result ("missed ..." when the
gripper closed on nothing). The target is sent in the base frame, so no localization is needed.
"""
import argparse

import rclpy
from rclpy.action import ActionClient

from roomrunner_interfaces.action import PickPlace


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--x", type=float, required=True, help="block x in the base frame (m)")
    parser.add_argument("--y", type=float, default=0.0, help="block y in the base frame (m)")
    parser.add_argument("--place", action="store_true", help="after the pick, put the block back down there")
    parser.add_argument("--name", default="test_block")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("try_pick")
    client = ActionClient(node, PickPlace, "pick_place")
    if not client.wait_for_server(timeout_sec=5.0):
        raise SystemExit("pick_place_server is not running.")

    def run(mode):
        goal = PickPlace.Goal(mode=mode, object_name=args.name)
        goal.target.header.frame_id = "base"           # base frame: no robot pose needed
        goal.target.pose.position.x, goal.target.pose.position.y = args.x, args.y
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
