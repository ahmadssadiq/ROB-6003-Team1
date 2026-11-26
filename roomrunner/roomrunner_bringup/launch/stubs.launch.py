"""RoomRunner with fake data: no camera, no robot, no LLM. Runs a whole mission end to end.

    ros2 launch roomrunner_bringup stubs.launch.py
    ros2 launch roomrunner_bringup stubs.launch.py request:="bring the blue block to the tray then go home"
    ros2 launch roomrunner_bringup stubs.launch.py request:=none      (send requests yourself:)
    ros2 topic pub --once /task_request std_msgs/msg/String "{data: 'bring the red block to the tray'}"

Stubs and what replaces them later:
    fake_overhead      (perception)    -> overhead camera, occupancy grid and object pose nodes
    fake_robot         (localization)  -> robot_node / sim_node + Assignment 04 particle filter + map_aruco_tf
    pick_place_server  (manipulation)  -> the same node, moving the arm through the Assignment 03 IK node
    task_parser in mock mode (planning) -> planner:=direct or planner:=llmp with an LLM key in .env
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def nodes(context):
    config = os.path.join(get_package_share_directory("roomrunner_bringup"), "config")
    room_file = os.path.join(config, "fake_room.yaml")
    with open(room_file) as f:
        start = yaml.safe_load(f)["robot_start"]
    with open(os.path.join(config, "robot.yaml")) as f:
        standoff = yaml.safe_load(f)["reach"]["standoff"]

    actions = [
        Node(package="roomrunner_perception", executable="fake_overhead", parameters=[{"room_file": room_file}]),
        Node(package="roomrunner_localization", executable="fake_robot",
             parameters=[{"start": [float(start["x"]), float(start["y"]), float(start["yaw"])]}]),
        Node(package="roomrunner_control", executable="waypoint_follower",
             parameters=[{"max_speed": 0.3, "align_with_overhead": False}]),   # no overhead robot tag in stub mode
        Node(package="roomrunner_planning", executable="path_planner"),
        Node(package="roomrunner_planning", executable="task_parser",
             parameters=[{"mode": LaunchConfiguration("planner").perform(context), "places_file": room_file}]),
        Node(package="roomrunner_planning", executable="mission_fsm",
             parameters=[{"places_file": room_file, "standoff": float(standoff)}]),
        Node(package="roomrunner_manipulation", executable="pick_place_server"),
    ]
    request = LaunchConfiguration("request").perform(context)
    if request and request != "none":
        actions.append(TimerAction(period=4.0, actions=[ExecuteProcess(
            cmd=["ros2", "topic", "pub", "--once", "-w", "1", "/task_request", "std_msgs/msg/String",
                 f"{{data: '{request}'}}"], output="log")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("planner", default_value="mock", description="task parser mode: mock, direct or llmp"),
        DeclareLaunchArgument("request", default_value="bring the red block to the tray",
                              description="request sent 4 s after start; none to send none"),
        OpaqueFunction(function=nodes),
    ])
