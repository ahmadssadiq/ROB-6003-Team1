from glob import glob

from setuptools import setup

package_name = "roomrunner_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aditya Mourya",
    description="The RoomRunner room in the course MuJoCo simulator, with a simulated overhead camera",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "room_sim = roomrunner_sim.room_sim:main",
        ],
    },
)
