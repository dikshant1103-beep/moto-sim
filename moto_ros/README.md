# moto_ros — moto_sim ↔ ROS 2 bridge (A2)

The spine of the ROS 2 layer for the Motorcycle Dashboard. A WebSocket *client*
node connects to the running PyChrono sim's telemetry (`ws://localhost:8765`) and
republishes the bike state as standard ROS 2 topics. **PyChrono stays the physics
and self-balancing engine** — this only exposes its state to the ROS ecosystem.

## Topics published
| Topic | Type | Contents |
|-------|------|----------|
| `/tf` | `odom → base_link` | pose (integrated x,y; orientation = roll/pitch/yaw) |
| `/moto/odom` | `nav_msgs/Odometry` | planar pose + body twist (vx, vy, yaw_rate) |
| `/moto/imu` | `sensor_msgs/Imu` | orientation, angular velocity, linear accel (ax, ay) |
| `/joint_states` | `sensor_msgs/JointState` | steer, fork/shock travel, wheel spin |

Frame convention: REP-103 (x fwd, y left, z up) — matches PyChrono directly.
Position has no absolute ground truth in the telemetry, so it's **integrated** from
body velocities (good for rviz/SLAM seeding; drifts over long runs as expected).

## Build & run
```bash
mkdir -p ~/ws_chassis/src && ln -s "$(pwd)/moto_ros" ~/ws_chassis/src/
cd ~/ws_chassis && source /opt/ros/humble/setup.bash
colcon build --packages-select moto_ros && source install/setup.bash

# terminal 1 — physics + telemetry
python moto_sim_main.py
# terminal 2 — the bridge
ros2 launch moto_ros bridge.launch.py        # or: ros2 run moto_ros ws_bridge
# terminal 3 — watch it
rviz2     # Fixed Frame = odom; add TF + Odometry (and later sensors)
ros2 topic echo /moto/imu
```
(`python3-websockets` required — `pip install websockets` if not present.)

## Why a bridge (not ROS inside the sim)
The PyChrono loop runs at 1 kHz; keeping it ROS-free avoids coupling and lets the
sim run standalone. The WebSocket is already the integration boundary the dashboard
uses, so the bridge reuses it. This is **A2** — the foundation for **A3**:
- **A3 sensors** — camera/lidar via either synthetic Three.js rendering (publish over
  rosbridge) or a Gazebo sensor-world fed by this odom/tf (Chrono physics, Gazebo render).
- **SLAM / multi-vehicle** — consume `/tf` + `/moto/odom` + sensor topics.
