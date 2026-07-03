"""bridge.launch.py — start the moto_sim → ROS 2 telemetry bridge.

    ros2 launch moto_ros bridge.launch.py ws_url:=ws://localhost:8765

Then open rviz2 (Fixed Frame = odom) and add TF + Odometry to watch the bike.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ws_url", default_value="ws://localhost:8765"),
        DeclareLaunchArgument("rate_hz", default_value="50.0"),
        Node(
            package="moto_ros", executable="ws_bridge", name="moto_ws_bridge",
            output="screen",
            parameters=[{
                "ws_url": LaunchConfiguration("ws_url"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                # Share Gazebo's /clock so TF stamps align with the lidar scan stamps
                # (SLAM drops scans otherwise). Harmless w/o Gazebo: stamps are 0 and
                # the dashboard views read pose, not time.
                "use_sim_time": True,
            }],
        ),
    ])
