"""nav2.launch.py — car-like autonomous navigation + obstacle avoidance for the bike.

Brings up the nav2 stack (controller/planner/behavior/bt_navigator + costmaps from
/moto/scan), the cmd_vel→moto adapter (so nav2's /cmd_vel drives PyChrono via lean),
and roam_goals (streams roaming goals so the bike explores while avoiding buildings).

Map→odom is provided by slam_toolbox (run slam too), odom→base_link by the bridge.

    # after: rosbridge, sim, bridge, world (roam OFF), slam
    ros2 launch moto_gazebo nav2.launch.py

EXPECT TUNING. If the bike steers the wrong way, flip cmd_vel_to_moto lean_sign.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


NAV2_NODES = [
    "controller_server", "planner_server", "behavior_server",
    "bt_navigator", "velocity_smoother",
]


def generate_launch_description():
    share = get_package_share_directory("moto_gazebo")
    params = os.path.join(share, "config", "nav2_params.yaml")
    nav2_share = get_package_share_directory("nav2_bt_navigator")
    default_bt = os.path.join(nav2_share, "behavior_trees",
                              "navigate_to_pose_w_replanning_and_recovery.xml")

    common = {"use_sim_time": True}

    nodes = [
        DeclareLaunchArgument("params_file", default_value=params),

        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[LaunchConfiguration("params_file")], remappings=[("cmd_vel", "cmd_vel")]),
        Node(package="nav2_planner", executable="planner_server", output="screen",
             parameters=[LaunchConfiguration("params_file")]),
        Node(package="nav2_behaviors", executable="behavior_server", output="screen",
             parameters=[LaunchConfiguration("params_file")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen",
             parameters=[LaunchConfiguration("params_file"),
                         {"use_sim_time": True, "default_nav_to_pose_bt_xml": default_bt}]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", output="screen",
             parameters=[LaunchConfiguration("params_file")],
             remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),

        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True, "node_names": NAV2_NODES}]),

        # nav2 /cmd_vel → PyChrono lean/speed
        Node(package="moto_gazebo", executable="cmd_vel_to_moto", name="cmd_vel_to_moto",
             output="screen", parameters=[common]),
        # roaming goal stream
        Node(package="moto_gazebo", executable="roam_goals", name="roam_goals",
             output="screen", parameters=[common]),
    ]
    return LaunchDescription(nodes)
