"""city.launch.py — full A3 sensor-world stack.

Brings up: city world (Gazebo) → spawn sensor_bike → pose_driver (drives the bike
from /moto/odom). Assumes moto_sim + moto_ros ws_bridge are already running so that
/moto/odom is being published.

    # terminal 1
    python moto_sim_main.py
    # terminal 2
    ros2 launch moto_ros bridge.launch.py
    # terminal 3
    ros2 launch moto_gazebo city.launch.py kind:=city headless:=false
    # then: rviz2  → add /moto/scan (LaserScan) + /moto/camera/image_raw (Image)

Args: kind:=city|straight  headless:=true|false
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _setup(context, *args, **kwargs):
    kind = LaunchConfiguration("kind").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"

    from moto_gazebo.city_world import generate_city, generate_straight_road
    from moto_gazebo.bike_model import sensor_bike_sdf, BIKE_COLORS
    world_path = os.path.join(tempfile.gettempdir(), f"moto_{kind}.world")
    sdf = generate_city() if kind == "city" else generate_straight_road()
    with open(world_path, "w") as f:
        f.write(sdf)

    # single sensor bike, namespace /moto (→ /moto/onboard_camera, /moto/scan)
    bike_sdf = os.path.join(tempfile.gettempdir(), "moto_sensor_bike.sdf")
    with open(bike_sdf, "w") as f:
        f.write(sensor_bike_sdf("sensor_bike", "moto", color=BIKE_COLORS[0]))

    gz_bin = "gzserver" if headless else "gazebo"
    return [
        ExecuteProcess(
            cmd=[gz_bin, "--verbose", world_path,
                 "-s", "libgazebo_ros_factory.so", "-s", "libgazebo_ros_init.so"],
            output="screen"),
        ExecuteProcess(
            cmd=["ros2", "run", "gazebo_ros", "spawn_entity.py",
                 "-entity", "sensor_bike", "-file", bike_sdf,
                 "-x", "0", "-y", "0", "-z", "0.0"],
            output="screen"),
        Node(package="moto_gazebo", executable="pose_driver", name="moto_pose_driver",
             output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("kind", default_value="city"),
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
