"""endless.launch.py — infinite roaming city.

Brings up: empty world (giant ground) → spawn the detailed sensor bike →
pose_driver (places it from /moto/odom) → city_streamer (spawns/deletes block
tiles around the bike forever) → roam_controller (auto-steers it around the road
grid). The bike rides the city endlessly and turns at intersections on its own.

Assumes moto_sim + ws_bridge are running (so /moto/odom is published).

    ros2 launch moto_gazebo endless.launch.py headless:=false
    # optional: ros2 launch moto_gazebo slam.launch.py   (live map of the explored city)
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    roam = LaunchConfiguration("roam").perform(context).lower() == "true"
    light = LaunchConfiguration("light").perform(context).lower() == "true"

    from moto_gazebo.city_world import generate_empty_world
    from moto_gazebo.bike_model import sensor_bike_sdf, BIKE_COLORS

    tmp = tempfile.gettempdir()
    world_path = os.path.join(tmp, "moto_endless.world")
    with open(world_path, "w") as f:
        f.write(generate_empty_world())
    bike_sdf = os.path.join(tmp, "moto_sensor_bike.sdf")
    with open(bike_sdf, "w") as f:
        f.write(sensor_bike_sdf("sensor_bike", "moto", color=BIKE_COLORS[0], light=light))
    stream_window = 1 if light else 2   # 3×3 vs 5×5 tiles

    gz_bin = "gzserver" if headless else "gazebo"
    actions = [
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
        Node(package="moto_gazebo", executable="city_streamer", name="city_streamer",
             output="screen", parameters=[{"window": stream_window}]),
    ]
    driver = LaunchConfiguration("driver").perform(context)
    if driver == "path":
        # KINEMATIC ride — publishes /moto/odom itself, cannot topple/stick.
        # Needs NO PyChrono sim or bridge. Still steered by Qwen/YOLO.
        actions.append(Node(package="moto_gazebo", executable="path_driver",
                            name="path_driver", output="screen"))
    elif roam:
        # PHYSICS roam (consumes /moto/odom from the bridge; can be unstable).
        actions.append(Node(package="moto_gazebo", executable="roam_controller",
                            name="roam_controller", output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("roam", default_value="true"),
        DeclareLaunchArgument("light", default_value="false"),
        DeclareLaunchArgument("driver", default_value="path"),  # path=kinematic (reliable) | physics
        OpaqueFunction(function=_setup),
    ])
