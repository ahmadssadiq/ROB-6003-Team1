from glob import glob

from setuptools import setup

package_name = "roomrunner_control"

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
    maintainer="Aditya Mourya",
    description="Waypoint follower for the LeKiwi base",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_follower = roomrunner_control.waypoint_follower:main",
        ],
    },
)
