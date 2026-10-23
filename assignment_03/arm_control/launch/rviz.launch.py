"""Show the arm in RViz. Start this next to whatever publishes arm_state.

Usage, with the real robot or the simulator already running in other terminals:
    ros2 launch arm_control rviz.launch.py

It starts three things:
    joint_state_bridge     arm_state (degrees) -> joint_states (radians)
    robot_state_publisher  joint_states + the robot description -> where every part is (tf)
    rviz2                  draws the robot
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("arm_control"))
    # The description names its mesh files relative to __MESH_DIR__. Fill in the real folder:
    # the course folder is the one that contains ros_ws/install/arm_control/share/arm_control.
    mesh_dir = share.resolve().parents[4] / "models" / "LeKiwi-sim" / "meshes"
    if not mesh_dir.is_dir():
        raise FileNotFoundError(f"Robot meshes not found in {mesh_dir}")
    description = (share / "urdf" / "lekiwi.urdf").read_text().replace("__MESH_DIR__", str(mesh_dir))

    return LaunchDescription([
        Node(package="arm_control", executable="joint_state_bridge", output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": description}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", str(share / "rviz" / "lekiwi.rviz")]),
    ])
