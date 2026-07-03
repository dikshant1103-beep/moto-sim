"""oval.launch.py — bounded OVAL race track, kinematic bike, continuous turning.

A fixed oval world (no streaming) so the bike stays in view and turns continuously
on the two corners. Kinematic path_driver follows the oval centerline → it CANNOT
topple and is always visibly cornering. Camera/lidar/SLAM/YOLO/Qwen all work.

    ros2 launch moto_gazebo oval.launch.py headless:=false light:=true
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    light = LaunchConfiguration("light").perform(context).lower() == "true"

    from moto_gazebo.oval_track import generate_oval_world, DEFAULT_LS, DEFAULT_RC
    from moto_gazebo.bike_model import sensor_bike_sdf, BIKE_COLORS

    tmp = tempfile.gettempdir()
    world_path = os.path.join(tmp, "moto_oval.world")
    with open(world_path, "w") as f:
        f.write(generate_oval_world())
    bike_sdf = os.path.join(tmp, "moto_sensor_bike.sdf")
    with open(bike_sdf, "w") as f:
        f.write(sensor_bike_sdf("sensor_bike", "moto", color=BIKE_COLORS[0], light=light))

    gz_bin = "gzserver" if headless else "gazebo"
    return [
        ExecuteProcess(
            cmd=[gz_bin, "--verbose", world_path,
                 "-s", "libgazebo_ros_factory.so", "-s", "libgazebo_ros_init.so"],
            output="screen"),
        ExecuteProcess(
            cmd=["ros2", "run", "gazebo_ros", "spawn_entity.py",
                 "-entity", "sensor_bike", "-file", bike_sdf, "-x", "0", "-y", "0", "-z", "0.1"],
            output="screen"),
        Node(package="moto_gazebo", executable="pose_driver", name="moto_pose_driver",
             output="screen"),
        Node(package="moto_gazebo", executable="path_driver", name="path_driver",
             output="screen", parameters=[{"track": "oval",
                                           "oval_straight": DEFAULT_LS,
                                           "oval_radius": DEFAULT_RC}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("light", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
