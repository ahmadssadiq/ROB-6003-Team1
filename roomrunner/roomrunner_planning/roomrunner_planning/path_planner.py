"""Path planner node: answers plan_path requests on the occupancy grid.

Service:
    plan_path       roomrunner_interfaces/PlanPath: start and goal pose (map) -> nav_msgs/Path (map)
Subscribes:
    occupancy_grid  nav_msgs/OccupancyGrid in the map frame

STUB: returns a straight line from start to goal, ignoring obstacles. A* on the inflated grid replaces it
in the next step of the planning module.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from roomrunner_interfaces.srv import PlanPath

STEP = 0.10   # m between path points


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        self.grid = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.on_grid, latched)
        self.create_service(PlanPath, "plan_path", self.plan)
        self.get_logger().info("Path planner ready (stub: straight lines).")

    def on_grid(self, msg):
        self.grid = msg

    def plan(self, request, response):
        (x0, y0), (x1, y1) = ((p.pose.position.x, p.pose.position.y) for p in (request.start, request.goal))
        length = math.hypot(x1 - x0, y1 - y0)
        count = max(1, math.ceil(length / STEP))
        response.path.header.frame_id = "map"
        response.path.header.stamp = self.get_clock().now().to_msg()
        for i in range(count + 1):
            pose = PoseStamped(header=response.path.header)
            pose.pose.position.x = x0 + (x1 - x0) * i / count
            pose.pose.position.y = y0 + (y1 - y0) * i / count
            pose.pose.orientation = request.goal.pose.orientation
            response.path.poses.append(pose)
        response.success, response.length = True, length
        response.message = "straight line (stub)"
        return response


def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
