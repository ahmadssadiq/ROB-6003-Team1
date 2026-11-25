"""Overhead camera on Mourya's laptop: publishes /occupancy_grid and /objects for the robot.

    ros2 launch roomrunner_bringup overhead.launch.py
Calibrate once first:  ros2 run roomrunner_perception calibrate_overhead --config <share>/config/overhead_camera.yaml
(edit the config in the source tree, roomrunner/roomrunner_bringup/config, then rebuild). See docs/networking.md.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("roomrunner_bringup"), "config")
    return LaunchDescription([Node(package="roomrunner_perception", executable="overhead_node", output="screen",
                                   parameters=[{"camera_file": os.path.join(config, "overhead_camera.yaml"),
                                                "objects_file": os.path.join(config, "objects.yaml")}])])
