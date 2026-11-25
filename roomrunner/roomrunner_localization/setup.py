from glob import glob

from setuptools import setup

package_name = "roomrunner_localization"

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
    description="Robot pose in the planning map: particle filter reuse (Assignment 04) and the ArUco map transform",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_robot = roomrunner_localization.fake_robot:main",
            "map_aruco_tf = roomrunner_localization.map_aruco_tf:main",
            "aruco_map_publisher = roomrunner_localization.aruco_map_publisher:main",
        ],
    },
)
