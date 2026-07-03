import os
from glob import glob
from setuptools import setup

package_name = "moto_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models", "sensor_bike"),
         glob("models/sensor_bike/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dikshant",
    maintainer_email="261631710+dikshant1103-beep@users.noreply.github.com",
    description="Gazebo sensor-world for moto_sim",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pose_driver = moto_gazebo.pose_driver:main",
            "convoy_ghost = moto_gazebo.convoy_ghost:main",
            "city_streamer = moto_gazebo.city_streamer:main",
            "roam_controller = moto_gazebo.roam_controller:main",
            "path_driver = moto_gazebo.path_driver:main",
            "cmd_vel_to_moto = moto_gazebo.cmd_vel_to_moto:main",
            "roam_goals = moto_gazebo.roam_goals:main",
            "make_world = moto_gazebo.make_world:main",
        ],
    },
)
