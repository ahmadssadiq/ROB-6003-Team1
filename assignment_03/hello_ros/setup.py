from setuptools import setup

package_name = "hello_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Your first ROS 2 nodes: a leader that publishes and a follower that subscribes",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            # name you type after "ros2 run hello_ros" = python file : function to start
            "leader = hello_ros.leader:main",
            "follower = hello_ros.follower:main",
        ],
    },
)
