"""multi_bike.launch.py — N-bike convoy in the Gazebo city sensor-world (A3+).

Lead bike (moto0) rides the real PyChrono trajectory (/moto/odom). The other
N−1 bikes are convoy ghosts: they follow the lead's recorded path with a growing
lag + alternating lane offset, each publishing /motoK/odom, each posed in Gazebo
by its own pose_driver. Lead carries camera+lidar; ghosts carry lidar only.

    # 1) physics + telemetry
    python moto_sim_main.py
    # 2) bridge → /moto/odom
    ros2 launch moto_ros bridge.launch.py
    # 3) city + N bikes (this file)
    ros2 launch moto_gazebo multi_bike.launch.py n:=3 kind:=city headless:=false
    # 4) rviz2 → add each /motoK/scan; Fixed Frame = odom

NOTE: every bike is teleported via the world's single /moto/set_entity_state.
The lead's odom is /moto/odom (from the bridge); ghosts get /motoK/odom.
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    n = int(LaunchConfiguration("n").perform(context))
    kind = LaunchConfiguration("kind").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    n = max(1, min(n, 5))

    from moto_gazebo.city_world import generate_city, generate_straight_road
    from moto_gazebo.bike_model import sensor_bike_sdf, BIKE_COLORS

    tmp = tempfile.gettempdir()
    world_path = os.path.join(tmp, f"moto_{kind}.world")
    with open(world_path, "w") as f:
        f.write(generate_city() if kind == "city" else generate_straight_road())

    gz_bin = "gzserver" if headless else "gazebo"
    actions = [ExecuteProcess(
        cmd=[gz_bin, "--verbose", world_path,
             "-s", "libgazebo_ros_factory.so", "-s", "libgazebo_ros_init.so"],
        output="screen")]

    for k in range(n):
        name = f"moto{k}"
        ns = f"moto{k}"
        is_lead = (k == 0)
        # lead odom comes from the bridge on /moto/odom; ghosts on /motoK/odom
        odom_topic = "/moto/odom" if is_lead else f"/{ns}/odom"
        color = BIKE_COLORS[k % len(BIKE_COLORS)]

        sdf = sensor_bike_sdf(name, ns, with_camera=is_lead, with_lidar=True, color=color)
        sdf_path = os.path.join(tmp, f"{name}.sdf")
        with open(sdf_path, "w") as f:
            f.write(sdf)

        # stagger initial spawn positions so they don't overlap before odom arrives
        actions.append(ExecuteProcess(
            cmd=["ros2", "run", "gazebo_ros", "spawn_entity.py",
                 "-entity", name, "-file", sdf_path,
                 "-x", str(-3.0 * k), "-y", "0", "-z", "0"],
            output="screen"))

        # ghost trajectory generator (followers only)
        if not is_lead:
            actions.append(Node(
                package="moto_gazebo", executable="convoy_ghost",
                name=f"ghost_{k}", output="screen",
                parameters=[{
                    "lead_odom": "/moto/odom",
                    "ghost_odom": odom_topic,
                    "lag_m": 8.0 * k,
                    "lane_offset_m": 1.8 * (1 if k % 2 else -1),
                }]))

        # pose driver: place this bike in Gazebo from its odom
        actions.append(Node(
            package="moto_gazebo", executable="pose_driver",
            name=f"pose_{k}", output="screen",
            parameters=[{
                "entity": name,
                "odom_topic": odom_topic,
                "state_service": "/moto/set_entity_state",
            }]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("n", default_value="3"),
        DeclareLaunchArgument("kind", default_value="city"),
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
