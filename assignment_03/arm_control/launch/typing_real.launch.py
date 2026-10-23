"""Start everything needed to type on a real keyboard with the REAL robot.

Usage, to measure key positions (motors off, move the arm by hand):
    ros2 launch arm_control typing_real.launch.py port:=/dev/serial/by-id/usb-...

Usage, to type (motors on, the arm moves by itself):
    ros2 launch arm_control typing_real.launch.py port:=/dev/serial/by-id/usb-... enable_motors:=true

Optional arguments:
    enable_motors:=true|false   (default false: read only, nothing moves)
    rviz:=true|false            (default true: also show the arm in RViz)
    max_step:=2.0               (default 5.0: the most a joint may move per step, in degrees.
                                 The robot takes 10 steps per second, so 2.0 means 20 deg/s.)

Then, in another terminal, read the fingertip position your forward kinematics computes:
    ros2 topic echo /fk_pose
or send the fingertip somewhere (only with enable_motors:=true):
    ros2 topic pub --once /ik_target geometry_msgs/msg/PoseStamped "{pose: {position: {x: 0.28, y: 0.0, z: 0.05}}}"
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_dir = Path(get_package_share_directory("arm_control")) / "launch"
    return LaunchDescription([
        DeclareLaunchArgument("port", description="serial port of LeKiwi's controller board"),
        DeclareLaunchArgument("enable_motors", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("max_step", default_value="5.0"),

        # The real robot takes the simulator's place. It uses the same topics.
        Node(package="lekiwi_ros", executable="robot_node", output="screen",
             parameters=[{"port": LaunchConfiguration("port"),
                          "enable_motors": ParameterValue(LaunchConfiguration("enable_motors"),
                                                          value_type=bool),
                          "max_relative_target": ParameterValue(LaunchConfiguration("max_step"),
                                                                value_type=float)}]),
        Node(package="arm_control", executable="forward_kinematics", output="screen"),
        Node(package="arm_control", executable="inverse_kinematics", output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / "rviz.launch.py")),
                                 condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
