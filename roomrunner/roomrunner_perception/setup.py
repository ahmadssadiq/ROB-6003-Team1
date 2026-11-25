from glob import glob

from setuptools import setup

package_name = "roomrunner_perception"

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
    description="Overhead camera: calibration, occupancy grid, object poses",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_overhead = roomrunner_perception.fake_overhead:main",
        ],
    },
)
