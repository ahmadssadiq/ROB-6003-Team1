"""RoomRunner on the real robot (runs on the mini PC). The overhead camera runs on Mourya's laptop
(overhead.launch.py); see docs/networking.md to connect the two.

    ros2 launch roomrunner_bringup robot.launch.py port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79017378-if00
    options: enable_motors:=true  camera_device:=0  planner:=mock|direct|llmp  arm:=stub|real
             pose_reset:=true|false  (re-seed the particle filter from the overhead camera when it is lost)

Reused unchanged: robot_node (course), Assignment 04 camera_node, aruco_detector, odometry_node,
localization_node (in the aruco_map frame) and the Assignment 03 inverse_kinematics (EEControl) node.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import yaml

ARGUMENTS = {"port": "", "enable_motors": "false", "camera_device": "0", "planner": "mock", "arm": "stub",
             "pose_reset": "true"}


def nodes(context):
    arg = {k: context.launch_configurations[k] for k in ARGUMENTS}
    if not arg["port"]:
        raise RuntimeError("give the motor board: port:=/dev/serial/by-id/...")
    config = os.path.join(get_package_share_directory("roomrunner_bringup"), "config")
    a4 = get_package_share_directory("aruco_localization")
    marker_map = os.path.join(a4, "config", "classroom_map.yaml")
    with open(marker_map) as f:
        marker_size = float(yaml.safe_load(f)["marker_size"])
    with open(os.path.join(config, "robot.yaml")) as f:
        robot = yaml.safe_load(f)
    cam = robot["camera_link"]
    places = os.path.join(config, "places.yaml")

    return [
        # Robot and front camera
        Node(package="lekiwi_ros", executable="robot_node", output="screen", parameters=[{
            "port": arg["port"], "enable_motors": arg["enable_motors"] == "true", "rate_hz": 20.0}]),
        Node(package="aruco_localization", executable="camera_node", parameters=[{
            "device": arg["camera_device"], "frame_id": "camera",
            "intrinsics_file": os.path.join(a4, "config", "camera_intrinsics.yaml")}]),
        Node(package="tf2_ros", executable="static_transform_publisher", arguments=[
            "--x", str(cam["x"]), "--y", str(cam["y"]), "--z", str(cam["z"]), "--pitch", str(-cam["pitch"]),
            "--frame-id", "base", "--child-frame-id", "camera_link"]),
        Node(package="tf2_ros", executable="static_transform_publisher", arguments=[
            "--roll", str(-math.pi / 2), "--yaw", str(-math.pi / 2), "--frame-id", "camera_link", "--child-frame-id", "camera"]),
        # Localization: Assignment 04 nodes, particle filter in the aruco_map frame
        Node(package="aruco_localization", executable="aruco_detector", parameters=[{"marker_size": marker_size}]),
        Node(package="aruco_localization", executable="odometry_node"),
        Node(package="aruco_localization", executable="localization_node",
             parameters=[os.path.join(a4, "config", "localization.yaml"), {"map_frame": "aruco_map"}]),
        Node(package="roomrunner_localization", executable="aruco_map_publisher", parameters=[{"map_file": marker_map}]),
        Node(package="roomrunner_localization", executable="map_aruco_tf",
             parameters=[{"transform_file": os.path.join(config, "T_map_aruco.yaml")}]),
        Node(package="roomrunner_localization", executable="pose_monitor", parameters=[{
            "transform_file": os.path.join(config, "T_map_aruco.yaml"), "reset": arg["pose_reset"] == "true"}]),
        # Control, planning, manipulation
        Node(package="roomrunner_control", executable="waypoint_follower"),
        Node(package="roomrunner_planning", executable="path_planner"),
        Node(package="roomrunner_planning", executable="task_parser",
             parameters=[{"mode": arg["planner"], "places_file": places}]),
        Node(package="roomrunner_planning", executable="mission_fsm",
             parameters=[{"places_file": places, "standoff": float(robot["reach"]["standoff"])}]),
        Node(package="arm_control", executable="inverse_kinematics"),
        Node(package="roomrunner_manipulation", executable="pick_place_server", parameters=[{
            "mode": arg["arm"], "settings_file": os.path.join(config, "manipulation.yaml")}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGUMENTS.items()]
                             + [OpaqueFunction(function=nodes)])
