"""ws_bridge.py — moto_sim WebSocket telemetry → ROS 2 bridge (A2 spine).

Connects as a WebSocket *client* to the running moto_sim telemetry server
(ws://localhost:8765) and republishes the bike state as standard ROS 2 topics:

    /tf            odom → base_link        (pose; orientation = roll/pitch/yaw)
    /moto/odom     nav_msgs/Odometry       (integrated 2D pose + body twist)
    /moto/imu      sensor_msgs/Imu         (orientation, angular vel, linear accel)
    /joint_states  sensor_msgs/JointState  (steer + suspension + wheel spin)

Design: the bridge is a SEPARATE process from the 1 kHz PyChrono sim — they talk
only over the existing WebSocket, so the physics stays ROS-free. PyChrono remains
the physics/balancing brain; this just exposes its state to the ROS ecosystem
(rviz, SLAM, multi-vehicle, sensors).

Frame: REP-103 (x forward, y left, z up) — matches PyChrono's convention directly.

Run:
    # terminal 1
    python moto_sim_main.py
    # terminal 2 (ROS sourced)
    ros2 run moto_ros ws_bridge          # or: python3 -m moto_ros.ws_bridge
    rviz2                                  # add TF + RobotModel/Odometry to watch it
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

try:
    # websockets >= 12 ships a blocking sync client — ideal for a dedicated thread
    # (no asyncio-in-thread fragility, and version-robust against the server).
    from websockets.sync.client import connect as ws_connect
except Exception:  # pragma: no cover
    ws_connect = None


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class WsBridge(Node):
    def __init__(self):
        super().__init__("moto_ws_bridge")
        self.declare_parameter("ws_url", os.environ.get("MOTO_WS_URL", "ws://localhost:8765"))
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("wheel_radius", 0.31)
        self.ws_url = self.get_parameter("ws_url").value
        self.rear_r = float(self.get_parameter("wheel_radius").value)

        qos = QoSProfile(depth=10)
        self.pub_odom = self.create_publisher(Odometry, "moto/odom", qos)
        self.pub_imu = self.create_publisher(Imu, "moto/imu", qos)
        self.pub_js = self.create_publisher(JointState, "joint_states", qos)
        self.tf = TransformBroadcaster(self)

        # latest telemetry frame (written by ws thread, read by timer)
        self._lock = threading.Lock()
        self._frame: dict | None = None
        # integrated odometry state
        self._x = 0.0
        self._y = 0.0
        self._last_t = None
        self._wheel_spin = 0.0

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._ws_thread, daemon=True)
        self._thread.start()
        self.get_logger().info(f"moto_ws_bridge → {self.ws_url}")

    # ── WebSocket client (background thread, blocking sync API) ──────────────
    def _ws_thread(self):
        if ws_connect is None:
            self.get_logger().error("'websockets>=12' not available — bridge idle. "
                                    "pip install --user -U websockets")
            return
        while not self._stop.is_set():
            try:
                with ws_connect(self.ws_url, open_timeout=5, max_size=None) as ws:
                    self.get_logger().info("connected to moto_sim telemetry")
                    for raw in ws:                       # blocks until next frame
                        try:
                            d = json.loads(raw)
                            if isinstance(d, dict) and "roll" in d:
                                with self._lock:
                                    self._frame = d
                        except (json.JSONDecodeError, TypeError):
                            pass
                        if self._stop.is_set():
                            break
            except Exception as e:  # sim not up yet / connection dropped
                self.get_logger().warn(f"ws disconnected ({type(e).__name__}); retrying…",
                                       throttle_duration_sec=5.0)
                time.sleep(1.0)

    # ── Publish (rclpy timer) ────────────────────────────────────────────────
    def _publish(self):
        with self._lock:
            f = self._frame
        if not f:
            return
        now = self.get_clock().now().to_msg()

        roll = f.get("roll", 0.0); pitch = f.get("pitch", 0.0); yaw = f.get("yaw", 0.0)
        vx = f.get("vx", 0.0); vy = f.get("vy", 0.0)
        t = f.get("t", None)

        # integrate planar odometry from body-frame velocities
        if self._last_t is not None and t is not None:
            dt = t - self._last_t
            if dt < 0:  # sim reset
                self._x = self._y = 0.0; dt = 0.0
            self._x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            self._y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            self._wheel_spin += (vx / max(self.rear_r, 0.05)) * dt
        self._last_t = t

        q = rpy_to_quat(roll, pitch, yaw)

        # TF: odom → base_link
        tfm = TransformStamped()
        tfm.header.stamp = now
        tfm.header.frame_id = "odom"
        tfm.child_frame_id = "base_link"
        tfm.transform.translation.x = self._x
        tfm.transform.translation.y = self._y
        tfm.transform.translation.z = self.rear_r
        tfm.transform.rotation = q
        self.tf.sendTransform(tfm)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = self.rear_r
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = f.get("yaw_rate", 0.0)
        self.pub_odom.publish(odom)

        # IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        imu.orientation = q
        imu.angular_velocity.x = f.get("roll_rate", 0.0)
        imu.angular_velocity.z = f.get("yaw_rate", 0.0)
        imu.linear_acceleration.x = f.get("ax", 0.0)
        imu.linear_acceleration.y = f.get("ay", 0.0)
        self.pub_imu.publish(imu)

        # Joint states
        js = JointState()
        js.header.stamp = now
        js.name = ["steering_joint", "front_susp", "rear_susp", "rear_axle", "front_axle"]
        js.position = [
            f.get("steer_angle", 0.0),
            -f.get("fork_travel", 0.0),
            -f.get("rear_travel", 0.0),
            self._wheel_spin,
            self._wheel_spin * (self.rear_r / 0.299),
        ]
        self.pub_js.publish(js)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
