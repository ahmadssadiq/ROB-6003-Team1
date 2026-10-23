"""Start everything needed to type on the simulated keyboard.

Usage:
    ros2 launch arm_control typing_sim.launch.py
    ros2 launch arm_control typing_sim.launch.py viewer:=false     (over SSH)

Then, in another terminal:
    ros2 topic pub --once /key std_msgs/msg/String "{data: a}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),

        Node(package="lekiwi_ros", executable="sim_node", output="screen",
             parameters=[{"scene": "keyboard",
                          "viewer": ParameterValue(LaunchConfiguration("viewer"), value_type=bool)}]),
        Node(package="arm_control", executable="forward_kinematics", output="screen"),
        Node(package="arm_control", executable="inverse_kinematics", output="screen"),
        Node(package="arm_control", executable="keyboard_typer", output="screen"),
    ])
