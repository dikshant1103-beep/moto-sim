# moto_gazebo — Gazebo sensor-world for moto_sim (A3)

Gives the bike a **camera + lidar in a city/road world** for SLAM and multi-vehicle,
**without moving physics out of PyChrono**. The split (your choice):

```
PyChrono (1 kHz physics + self-balancing)
   │  WebSocket telemetry
   ▼
moto_ros ws_bridge ──► /moto/odom, /tf, /moto/imu
   │
   ▼
moto_gazebo pose_driver ──► teleports kinematic `sensor_bike` to that pose
   │
   ▼
Gazebo city world renders ONBOARD CAMERA + LIDAR
   └──► /moto/camera/image_raw, /moto/scan  ──►  SLAM / rviz / multi-vehicle
```

Gazebo here is a **sensor renderer only** — the bike model is kinematic (`gravity off`,
teleported each frame), so there is **no balancing problem** in Gazebo. The physics you
trust stays in Chrono.

## Topics produced
| Topic | Type | From |
|-------|------|------|
| `/moto/camera/image_raw` | `sensor_msgs/Image` | onboard camera (rider POV, 640×480, 30 Hz) |
| `/moto/camera/camera_info` | `sensor_msgs/CameraInfo` | camera intrinsics |
| `/moto/scan` | `sensor_msgs/LaserScan` | 360° 2D lidar, 60 m, 20 Hz |

## Worlds
- **city** — `blocks×blocks` road grid lined with buildings (SLAM landmarks). Procedural, seeded.
- **straight** — a single long road with roadside buildings (fast first bring-up).

Generate standalone: `python3 -m moto_gazebo.make_world --kind city --out /tmp/city.world`

## Build & run
```bash
ln -s "$(pwd)/moto_gazebo" ~/ws_chassis/src/   # alongside moto_ros
cd ~/ws_chassis && source /opt/ros/humble/setup.bash
colcon build --packages-select moto_ros moto_gazebo && source install/setup.bash

python moto_sim_main.py                          # 1) physics + telemetry
ros2 launch moto_ros bridge.launch.py            # 2) → /moto/odom
ros2 launch moto_gazebo city.launch.py kind:=city headless:=false   # 3) world+sensors
# 4) visualize / process:
rviz2     # add Image (/moto/camera/image_raw) + LaserScan (/moto/scan), Fixed Frame=odom
```

## SLAM (done)
```bash
ros2 launch moto_gazebo slam.launch.py      # needs ros-humble-slam-toolbox
rviz2   # Fixed Frame = map; add Map (/map) + LaserScan (/moto/scan)
```
`config/slam_toolbox.yaml` (online async, loop-closure on) + `launch/slam.launch.py`
(publishes the static base_link→lidar_link/camera_link TF, then slam_toolbox).

## Multi-vehicle (done)
```bash
ros2 launch moto_gazebo multi_bike.launch.py n:=3 kind:=city headless:=false
rviz2   # add /moto0/scan, /moto1/scan, …
```
- **Lead (moto0)** rides the real PyChrono trajectory (`/moto/odom`), camera+lidar.
- **Ghosts (moto1..)** = `convoy_ghost` nodes that replay the lead's recorded path with a
  growing lag + alternating lane offset, publishing `/motoK/odom`; lidar only.
- Each bike is placed by its own `pose_driver` via the world's `/moto/set_entity_state`.
- **Honest scope:** ghosts are kinematic followers of recorded real trajectory, not
  independently simulated dynamics — good for sensor/SLAM/scene multi-vehicle, not bike-to-bike
  dynamics. (True multi-physics would need N PyChrono sims, each with its own bridge.)

## Honesty notes
- Pose is integrated odometry from body velocities (no absolute ground truth in telemetry),
  so over long runs it drifts — that's exactly what SLAM is for (loop closure corrects it).
- Camera/lidar are Gazebo Classic `gazebo_ros` plugins; needs `ros-humble-gazebo-ros-pkgs`.
- Not run end-to-end in this session (no display); SDF/world XML validated well-formed +
  Python compiled. First on-machine run may need plugin `.so` availability checks.
