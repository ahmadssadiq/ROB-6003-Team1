"""RoomRunner in the course MuJoCo simulator: the real localization, perception, planning and control nodes,
with a simulated robot, room and overhead camera (roomrunner_sim). Run it in the conda environment.

    ros2 launch roomrunner_bringup sim.launch.py
    ros2 launch roomrunner_bringup sim.launch.py request:="bring the red cuboid to the tray then go home"
    options: viewer:=true (MuJoCo window)  planner:=mock|direct|llmp  request:=none  odometry_noise:=0.05

What runs (simulator side -> the same nodes as on the real robot):
    room_sim            the course sim_node with our room added: front camera, odometry, overhead camera
    aruco_detector, odometry_node, localization_node   Assignment 04, the particle filter in aruco_map
    aruco_map_publisher, map_aruco_tf, pose_monitor   roomrunner_localization (T_map_aruco = identity here)
    overhead_node       roomrunner_perception, pictures from overhead/image instead of a USB camera
    path_planner, task_parser, mission_fsm, waypoint_follower   planning and control
    pick_place_server   stub mode (the arm does not move yet)

The simulator's overhead camera is known exactly, so its calibration files are written by
roomrunner_sim.sim_config into ~/.ros/roomrunner_sim at every start (no calibrate_overhead needed).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node

ARGUMENTS = {"viewer": "false", "planner": "mock", "odometry_noise": "0.05",
             "request": "bring the red cuboid to the tray then go home"}
OUT = os.path.expanduser("~/.ros/roomrunner_sim")


def nodes(context):
    from roomrunner_sim.sim_config import load_room, write_configs
    arg = {k: context.launch_configurations[k] for k in ARGUMENTS}
    bringup = os.path.join(get_package_share_directory("roomrunner_bringup"), "config")
    sim_share = os.path.join(get_package_share_directory("roomrunner_sim"), "config")
    a4 = get_package_share_directory("aruco_localization")
    marker_map = os.path.join(a4, "config", "sim_room_map.yaml")
    with open(marker_map) as f:
        markers = yaml.safe_load(f)
    room_file = os.path.join(sim_share, "sim_room.yaml")
    room = load_room(room_file)
    files = write_configs(room, os.path.join(bringup, "objects.yaml"), os.path.join(sim_share, "places_sim.yaml"), OUT)
    transform = os.path.join(OUT, "T_map_aruco.yaml")
    with open(transform, "w") as f:      # the simulator's world is the wall-marker frame: exactly the identity
        yaml.safe_dump({"T_map_aruco": {"x": 0.0, "y": 0.0, "yaw": 0.0}, "measured": True}, f)
    with open(os.path.join(bringup, "robot.yaml")) as f:
        standoff = yaml.safe_load(f)["reach"]["standoff"]

    actions = [
        Node(package="roomrunner_sim", executable="room_sim", output="screen", parameters=[{
            "room_file": room_file, "objects_file": os.path.join(bringup, "objects.yaml"),
            "empty_floor_file": os.path.join(OUT, "overhead_empty_floor.png"),
            "scene": "aruco", "map_file": marker_map, "viewer": arg["viewer"] == "true",
            "start_pose": [float(v) for v in room["start_pose"]], "publish_images": True, "camera": "front",
            "camera_frame": "camera", "image_width": 640, "image_height": 480,
            "odometry_noise": float(arg["odometry_noise"])}]),
        # the simulated front camera's mount (as in Assignment 04's sim launch)
        Node(package="tf2_ros", executable="static_transform_publisher", arguments=[
            "--x", "0.125", "--y", "0.0", "--z", "0.03", "--frame-id", "base", "--child-frame-id", "camera_link"]),
        Node(package="tf2_ros", executable="static_transform_publisher", arguments=[
            "--roll", "-1.5707963", "--yaw", "-1.5707963", "--frame-id", "camera_link", "--child-frame-id", "camera"]),
        Node(package="aruco_localization", executable="aruco_detector", parameters=[{
            "dictionary": markers.get("dictionary", "DICT_4X4_50"), "marker_size": float(markers["marker_size"])}]),
        Node(package="aruco_localization", executable="odometry_node"),
        Node(package="aruco_localization", executable="localization_node",
             parameters=[os.path.join(a4, "config", "localization.yaml"), {"map_frame": "aruco_map"}]),
        Node(package="roomrunner_localization", executable="aruco_map_publisher", parameters=[{"map_file": marker_map}]),
        Node(package="roomrunner_localization", executable="map_aruco_tf", parameters=[{"transform_file": transform}]),
        Node(package="roomrunner_localization", executable="pose_monitor", parameters=[{"transform_file": transform}]),
        Node(package="roomrunner_perception", executable="overhead_node", output="screen", parameters=[{
            "camera_file": str(files["overhead"]), "objects_file": str(files["objects"]),
            "image_topic": "overhead/image"}]),
        # the base is 0.15 m in radius; the rest is margin for the particle filter's error (about 0.1 to 0.2 m)
        Node(package="roomrunner_planning", executable="path_planner", parameters=[{"robot_radius": 0.30}]),
        Node(package="roomrunner_planning", executable="task_parser",
             parameters=[{"mode": arg["planner"], "places_file": str(files["places"])}]),
        Node(package="roomrunner_planning", executable="mission_fsm", output="screen",
             parameters=[{"places_file": str(files["places"]), "standoff": float(standoff)}]),
        Node(package="roomrunner_control", executable="waypoint_follower"),
        Node(package="roomrunner_manipulation", executable="pick_place_server"),
    ]
    if arg["request"] not in ("", "none"):
        actions.append(TimerAction(period=20.0, actions=[ExecuteProcess(   # after the filter has found the robot
            cmd=["ros2", "topic", "pub", "--once", "-w", "1", "/task_request", "std_msgs/msg/String",
                 f"{{data: '{arg['request']}'}}"], output="log")]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGUMENTS.items()]
                             + [OpaqueFunction(function=nodes)])
