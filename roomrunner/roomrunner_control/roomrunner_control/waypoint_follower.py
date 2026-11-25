"""Waypoint follower: drives the LeKiwi base along a path and stops at its end, aligned with its heading.

Action server:
    navigate_to    roomrunner_interfaces/NavigateTo: path (map frame) and tolerances
Subscribes:
    robot_pose     geometry_msgs/PoseStamped in the map frame
Publishes:
    base_commands  std_msgs/Float64MultiArray [forward m/s, left m/s, turn deg/s] (robot_node / sim_node units)
Parameters:
    max_speed      m/s (default 0.15)
    max_turn       rad/s (default 0.5)
    gain           1/s, proportional gain on the position error (default 1.5)
    yaw_gain       1/s, proportional gain on the heading error (default 1.5)
    lookahead      m: move on to the next waypoint once this close (default 0.15)
    timeout        s: give up after this long (default 120)

The base is omnidirectional, so it drives straight at the next waypoint in any direction while it turns
towards the goal heading. This is a simple starting point (owner: Mourya): the final alignment with the
overhead camera is added later.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from roomrunner_interfaces.action import NavigateTo

RATE_HZ = 20.0
SETTLE_TIME = 0.5   # s the robot must stay inside the tolerances before it counts as stopped


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")
        for name, default in (("max_speed", 0.15), ("max_turn", 0.5), ("gain", 1.5), ("yaw_gain", 1.5),
                              ("lookahead", 0.15), ("timeout", 120.0)):
            self.declare_parameter(name, default)
        self.pose = None
        self.lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self.create_subscription(PoseStamped, "robot_pose", self.on_pose, 10, callback_group=group)
        self.command_pub = self.create_publisher(Float64MultiArray, "base_commands", 10)
        self.server = ActionServer(self, NavigateTo, "navigate_to", self.execute, callback_group=group)
        self.get_logger().info("Waypoint follower ready.")

    def on_pose(self, msg):
        with self.lock:
            self.pose = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose))

    def send(self, forward, left, turn):
        # /base_commands takes the turn rate in deg/s.
        self.command_pub.publish(Float64MultiArray(data=[float(forward), float(left), math.degrees(turn)]))

    def execute(self, goal_handle):
        goal = goal_handle.request
        points = [(p.pose.position.x, p.pose.position.y) for p in goal.path.poses]
        result = NavigateTo.Result()
        if not points:
            goal_handle.abort()
            result.message = "empty path"
            return result
        goal_yaw = yaw_of(goal.path.poses[-1].pose)
        p = {name: self.get_parameter(name).value for name in
             ("max_speed", "max_turn", "gain", "yaw_gain", "lookahead", "timeout")}
        index, start, settled_since = 0, time.monotonic(), None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.send(0, 0, 0)
                goal_handle.canceled()
                result.message = "canceled"
                return result
            if time.monotonic() - start > p["timeout"]:
                break
            with self.lock:
                pose = self.pose
            if pose is None:
                time.sleep(1.0 / RATE_HZ)
                continue
            x, y, yaw = pose
            # Skip waypoints that are already close, except the last one.
            while index < len(points) - 1 and math.dist((x, y), points[index]) < p["lookahead"]:
                index += 1
            tx, ty = points[index]
            dx, dy = tx - x, ty - y
            distance = math.hypot(dx, dy)
            yaw_error = wrap(goal_yaw - yaw)

            at_goal = (index == len(points) - 1 and distance < goal.position_tolerance
                       and abs(yaw_error) < goal.yaw_tolerance)
            if at_goal:
                self.send(0, 0, 0)
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= SETTLE_TIME:
                    goal_handle.succeed()
                    result.success = True
                    result.message = "aligned and stopped"
                    result.final_position_error, result.final_yaw_error = distance, yaw_error
                    return result
            else:
                settled_since = None
                speed = min(p["max_speed"], p["gain"] * distance)
                vx, vy = (speed * dx / distance, speed * dy / distance) if distance > 1e-6 else (0.0, 0.0)
                # Map-frame velocity into the robot's frame.
                forward = math.cos(yaw) * vx + math.sin(yaw) * vy
                left = -math.sin(yaw) * vx + math.cos(yaw) * vy
                turn = max(-p["max_turn"], min(p["max_turn"], p["yaw_gain"] * yaw_error))
                self.send(forward, left, turn)

            feedback = NavigateTo.Feedback()
            feedback.distance_remaining = distance + sum(
                math.dist(points[i], points[i + 1]) for i in range(index, len(points) - 1))
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0 / RATE_HZ)

        self.send(0, 0, 0)
        goal_handle.abort()
        result.message = "timed out"
        return result


def main():
    rclpy.init()
    node = WaypointFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
