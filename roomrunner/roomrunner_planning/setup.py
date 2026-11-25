from glob import glob

from setuptools import setup

package_name = "roomrunner_planning"

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
    maintainer="Ahmad Sadiq",
    description="Task parser, A* path planner and mission state machine",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_parser = roomrunner_planning.task_parser:main",
            "path_planner = roomrunner_planning.path_planner:main",
            "mission_fsm = roomrunner_planning.mission_fsm:main",
        ],
    },
)
