# ros2_stack — live ROS 2 / Gazebo / SLAM control from the dashboard

The **Robotics (ROS 2)** tab in the Motorcycle Dashboard starts/stops the whole
stack and shows it live, so you don't need separate terminals + Gazebo + rviz2.

```
Dashboard (Electron)
  ├─ Stack Control  ──IPC──► moto_ros_launch.sh <component>   (spawns real ROS 2 procs)
  └─ Live Views     ──ws──►  rosbridge (:9090)  ──► SLAM map, lidar, camera, fleet
```

## One-time setup
```bash
# 1) colcon workspace with the two packages
mkdir -p ~/ws_chassis/src
ln -s "$(pwd)/../moto_ros"    ~/ws_chassis/src/
ln -s "$(pwd)/../moto_gazebo" ~/ws_chassis/src/
cd ~/ws_chassis && source /opt/ros/humble/setup.bash && colcon build

# 2) runtime deps
sudo apt install ros-humble-rosbridge-suite ros-humble-slam-toolbox \
                 ros-humble-gazebo-ros-pkgs ros-humble-image-transport-plugins
```
If your workspace isn't `~/ws_chassis`, set `MOTO_WS` before launching the dashboard
(e.g. `MOTO_WS=~/my_ws npm start`). ROS distro override: `ROS_SETUP=/opt/ros/<distro>/setup.bash`.

## Use
1. `cd dashboard && npm start`  → open the **Robotics (ROS 2)** tab.
2. Click **Start** in order: **rosbridge → sim → bridge → world → slam**.
   (Use **Multi-vehicle** *instead of* world for a 3-bike convoy.)
3. Live views populate: SLAM map (`/map`), lidar (`/moto/scan`), onboard camera, fleet top-down.

## Components (`moto_ros_launch.sh <component>`)
| id | runs |
|----|------|
| `rosbridge` | `rosbridge_server` websocket (:9090) — feeds the dashboard views |
| `sim` | `python3 moto_sim_main.py` — PyChrono physics + balancing + telemetry |
| `bridge` | `moto_ros` ws_bridge → `/moto/odom`, `/tf`, `/moto/imu` |
| `world` | `moto_gazebo` city world + sensor bike (camera + lidar) |
| `slam` | `slam_toolbox` 2D mapping |
| `multi` | N-bike convoy (default n=3) |

## Honest notes
- Each component is a real ROS 2 process spawned in its own process group; **Stop** sends
  SIGINT then SIGKILL to the whole group (so `ros2 launch` children die too).
- Camera view uses `/moto/camera/image_raw/compressed` — needs `image-transport-plugins`.
- The dashboard's launch buttons require the **Electron** app (`npm start`); in a plain
  browser the live views still work if you start the stack from a terminal.
- Not yet run end-to-end here (no display in the build session) — code/build verified.
