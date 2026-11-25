"""Path planner node: answers plan_path requests with A* on the occupancy grid.

Service:
    plan_path       roomrunner_interfaces/PlanPath: start and goal pose (map) -> nav_msgs/Path (map)
Subscribes:
    occupancy_grid  nav_msgs/OccupancyGrid in the map frame (0 free; 100 occupied and -1 unknown are blocked)
Parameters:
    robot_radius    m: obstacles are grown by this much (default 0.20)
    waypoint_step   keep every n-th grid cell of the A* path as a waypoint (default 4)

The goal is often an approach pose next to an object; if it lies inside the grown obstacles, the
nearest free cell is used instead. Every waypoint's orientation is the goal's, so the follower turns
towards the final heading as it drives.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from roomrunner_interfaces.srv import PlanPath
from roomrunner_planning.astar import astar, cell_to_world, inflate, nearest_free, path_length, world_to_cell


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("waypoint_step", 4)
        self.grid = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.on_grid, latched)
        self.create_service(PlanPath, "plan_path", self.plan)
        self.get_logger().info("Path planner ready (A*).")

    def on_grid(self, msg):
        info = msg.info
        occupied = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width) != 0
        radius = self.get_parameter("robot_radius").value / info.resolution
        self.grid = {"blocked": inflate(occupied, radius), "resolution": info.resolution,
                     "origin": (info.origin.position.x, info.origin.position.y)}

    def plan(self, request, response):
        if self.grid is None:
            response.success, response.message = False, "no occupancy grid received yet"
            return response
        blocked, res, origin = self.grid["blocked"], self.grid["resolution"], self.grid["origin"]
        start = world_to_cell(request.start.pose.position.x, request.start.pose.position.y, origin, res)
        goal = world_to_cell(request.goal.pose.position.x, request.goal.pose.position.y, origin, res)
        start, goal = nearest_free(blocked, start), nearest_free(blocked, goal)
        cells = astar(blocked, start, goal) if start and goal else None
        if cells is None:
            response.success, response.message = False, "no path"
            return response

        step = max(1, self.get_parameter("waypoint_step").value)
        keep = cells[::step] if cells[-1] in cells[::step] else cells[::step] + [cells[-1]]
        response.path.header.frame_id = "map"
        response.path.header.stamp = self.get_clock().now().to_msg()
        for row, col in keep:
            pose = PoseStamped(header=response.path.header)
            pose.pose.position.x, pose.pose.position.y = cell_to_world(row, col, origin, res)
            pose.pose.orientation = request.goal.pose.orientation
            response.path.poses.append(pose)
        response.success, response.length = True, path_length(cells, res)
        response.message = f"A*: {len(cells)} cells, {response.length:.2f} m"
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
