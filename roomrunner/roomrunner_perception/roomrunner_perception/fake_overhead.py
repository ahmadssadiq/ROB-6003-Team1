"""Stand-in for the overhead camera: publishes a made-up room so the rest of the system can run.

Publishes:
    occupancy_grid  nav_msgs/OccupancyGrid in the map frame: 0 free, 100 occupied
    objects         roomrunner_interfaces/ObjectPoseArray in the map frame
Parameters:
    room_file       YAML with the room size, obstacles and objects (roomrunner_bringup/config/fake_room.yaml)

The real perception nodes publish the same two topics from the overhead camera image.
Both are latched (transient local), so a node that starts later still receives them.
"""
import math

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from roomrunner_interfaces.msg import ObjectPose, ObjectPoseArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def room_grid(room):
    """The room as a 2-D array of 0 (free) and 100 (occupied), row index = y, column index = x."""
    resolution = room["resolution"]
    rows, cols = round(room["height"] / resolution), round(room["width"] / resolution)
    grid = np.zeros((rows, cols), dtype=np.int8)
    wall = max(1, round(room["wall_thickness"] / resolution))
    grid[:wall, :] = grid[-wall:, :] = grid[:, :wall] = grid[:, -wall:] = 100
    for x_min, y_min, x_max, y_max in room["obstacles"]:
        grid[int(y_min / resolution):math.ceil(y_max / resolution),
             int(x_min / resolution):math.ceil(x_max / resolution)] = 100
    return grid


class FakeOverhead(Node):
    def __init__(self):
        super().__init__("fake_overhead")
        self.declare_parameter("room_file", "")
        with open(self.get_parameter("room_file").value) as f:
            config = yaml.safe_load(f)

        room = config["room"]
        grid = room_grid(room)
        self.grid_msg = OccupancyGrid()
        self.grid_msg.header.frame_id = "map"
        self.grid_msg.info.resolution = float(room["resolution"])
        self.grid_msg.info.height, self.grid_msg.info.width = grid.shape
        self.grid_msg.info.origin.orientation.w = 1.0
        self.grid_msg.data = grid.flatten().tolist()

        self.objects_msg = ObjectPoseArray()
        self.objects_msg.header.frame_id = "map"
        for item in config["objects"]:
            obj = ObjectPose(name=item["name"], kind=item["kind"], tag_id=item["tag_id"], confidence=1.0)
            obj.pose.position.x, obj.pose.position.y = float(item["x"]), float(item["y"])
            obj.pose.orientation.z, obj.pose.orientation.w = math.sin(item["yaw"] / 2), math.cos(item["yaw"] / 2)
            self.objects_msg.objects.append(obj)

        self.grid_pub = self.create_publisher(OccupancyGrid, "occupancy_grid", LATCHED)
        self.objects_pub = self.create_publisher(ObjectPoseArray, "objects", LATCHED)
        self.publish()
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f"Fake room {room['width']} x {room['height']} m with objects "
                               f"{[o.name for o in self.objects_msg.objects]}")

    def publish(self):
        now = self.get_clock().now().to_msg()
        self.grid_msg.header.stamp = self.objects_msg.header.stamp = now
        self.grid_pub.publish(self.grid_msg)
        self.objects_pub.publish(self.objects_msg)


def main():
    rclpy.init()
    node = FakeOverhead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
