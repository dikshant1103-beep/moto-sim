"""slam.launch.py — 2D SLAM over the city sensor-world (slam_toolbox).

Builds a live occupancy map from /moto/scan as the bike rides the city. Run AFTER
the sim + bridge + city world are up:

    python moto_sim_main.py
    ros2 launch moto_ros bridge.launch.py
    ros2 launch moto_gazebo city.launch.py kind:=city
    ros2 launch moto_gazebo slam.launch.py
    rviz2     # add Map (/map) + LaserScan (/moto/scan), Fixed Frame = map

TF chain produced:
    map → odom        (slam_toolbox)
    odom → base_link  (moto_ros ws_bridge)
    base_link → lidar_link / camera_link  (static, here — must match model.sdf poses)

Needs:  sudo apt install ros-humble-slam-toolbox
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("moto_gazebo")
    slam_cfg = os.path.join(share, "config", "slam_toolbox.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("slam_params_file", default_value=slam_cfg),

        # Static sensor frames — MUST match models/sensor_bike/model.sdf sensor poses.
        # args: x y z yaw pitch roll parent child  (use_sim_time so stamps share /clock)
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="tf_lidar", parameters=[{"use_sim_time": True}],
             arguments=["0.1", "0", "1.0", "0", "0", "0", "base_link", "lidar_link"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="tf_camera", parameters=[{"use_sim_time": True}],
             arguments=["1.45", "0", "1.05", "0", "0.12", "0", "base_link", "camera_link"]),

        # slam_toolbox — online async mapping
        Node(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[LaunchConfiguration("slam_params_file")],
        ),
    ])
