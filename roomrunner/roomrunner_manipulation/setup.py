from glob import glob

from setuptools import setup

package_name = "roomrunner_manipulation"

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
    maintainer="Aditya Sarvanagupta",
    description="SO-101 pick and place on top of the Assignment 03 EEControl node",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pick_place_server = roomrunner_manipulation.pick_place_server:main",
        ],
    },
)
