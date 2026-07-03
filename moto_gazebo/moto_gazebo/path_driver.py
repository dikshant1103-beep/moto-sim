"""path_driver.py — kinematic city ride (cannot topple, always moves).

Replaces the PyChrono→bridge odom source for the "ride the city" use case. Instead
of a fragile self-balancing closed loop (which kept toppling), this integrates the
bike's pose KINEMATICALLY at constant speed with a bounded turn rate, so it ALWAYS
moves smoothly through the city and can NEVER fall over or get stuck. Camera/lidar/
SLAM/YOLO/Qwen all work because they only need the bike to move smoothly.

It still roams the road grid and is STILL steered by the AI:
  • Qwen /moto/decision  → biases which way it turns at intersections
  • YOLO /moto/yolo/avoid → nudges heading to dodge detected obstacles

Publishes /moto/odom (pose: x,y,yaw + visual lean roll; twist: v, yaw_rate) at 50 Hz,
which pose_driver consumes to place the sensor_bike. The PyChrono physics stays in
the dashboard's Live tab (straight-line, where it's solid).

Frame: REP-103 (x fwd, y left, z up).
"""

from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String, Bool, Empty
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from moto_gazebo.city_world import TILE_SIZE
from moto_gazebo import oval_track

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]
G = 9.81


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PathDriver(Node):
    def __init__(self):
        super().__init__("path_driver")
        self.declare_parameter("track", "oval")       # oval | grid
        self.declare_parameter("tile_size", TILE_SIZE)
        self.declare_parameter("speed", 8.0)          # m/s cruise
        self.declare_parameter("turn_radius", 14.0)   # m min radius (grid mode)
        self.declare_parameter("oval_straight", oval_track.DEFAULT_LS)
        self.declare_parameter("oval_radius", oval_track.DEFAULT_RC)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("wheel_radius", 0.31)
        self.track = self.get_parameter("track").value
        self.T = float(self.get_parameter("tile_size").value)
        self.v = float(self.get_parameter("speed").value)
        self.R = float(self.get_parameter("turn_radius").value)
        self.Ls = float(self.get_parameter("oval_straight").value)
        self.Rc = float(self.get_parameter("oval_radius").value)
        self.rear_r = float(self.get_parameter("wheel_radius").value)
        self.w_max = self.v / self.R                  # max yaw rate (grid)

        # kinematic state
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.wheel_spin = 0.0
        self.s = 0.0                                   # arc length (oval mode)
        self._dir = (1, 0)
        self._target = (self.T, 0.0)                  # next intersection ahead (grid)
        self.qwen = None
        self.yolo_avoid = 0.0

        self.paused = False
        self.pub = self.create_publisher(Odometry, "moto/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, "/moto/decision", self._qwen, 10)
        self.create_subscription(Float64, "/moto/yolo/avoid", self._yolo, 10)
        # live ride controls from the dashboard (over rosbridge)
        self.create_subscription(Float64, "/moto/ride/speed", self._set_speed, 10)
        self.create_subscription(Bool, "/moto/ride/pause", self._set_pause, 10)
        self.create_subscription(Empty, "/moto/ride/reset", self._do_reset, 10)

        rate = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"path_driver: track={self.track}, v={self.v} m/s (kinematic, "
                               f"cannot topple). oval Ls={self.Ls} Rc={self.Rc} / grid T={self.T}")

    def _qwen(self, m: String):
        d = m.data.strip().lower()
        if d in ("left", "right", "straight"):
            self.qwen = d

    def _yolo(self, m: Float64):
        self.yolo_avoid = float(m.data)

    def _set_speed(self, m: Float64):
        self.v = max(0.0, min(25.0, float(m.data)))
        self.w_max = self.v / self.R if self.R > 0.1 else 0.5

    def _set_pause(self, m: Bool):
        self.paused = bool(m.data)

    def _do_reset(self, _m: Empty):
        self.s = 0.0
        self.x = self.y = self.yaw = 0.0

    def _pick_dir(self):
        back = (-self._dir[0], -self._dir[1])
        choices = [d for d in DIRS if d != back]
        weights = [0.55 if d == self._dir else 0.225 for d in choices]
        if self.qwen:                                  # Qwen strategic bias
            fx, fy = self._dir
            want = {"straight": self._dir, "left": (-fy, fx), "right": (fy, -fx)}[self.qwen]
            for i, d in enumerate(choices):
                if d == want:
                    weights[i] *= 5.0
            self.qwen = None
        self._dir = random.choices(choices, weights=weights)[0]

    def _tick_oval(self):
        """Follow the oval centerline by arc length → continuous banked turning."""
        self.s += self.v * self.dt
        x, y, heading, curv = oval_track.sample(self.s, self.Ls, self.Rc)
        self.x, self.y, self.yaw = x, y, heading
        return self.v * curv   # yaw rate = v·curvature (0 on straights, v/Rc on corners)

    def _tick_grid(self):
        # reached the next intersection? choose a new direction + next target
        gx, gy = self._target
        if math.hypot(gx - self.x, gy - self.y) < self.v * self.dt * 2 + 0.5:
            self.x, self.y = gx, gy
            self._pick_dir()
            self._target = (gx + self._dir[0] * self.T, gy + self._dir[1] * self.T)
            gx, gy = self._target
        bearing = math.atan2(gy - self.y, gx - self.x)
        err = wrap(bearing - self.yaw) - 0.4 * self.yolo_avoid
        yaw_rate = max(-self.w_max, min(self.w_max, err / self.dt))
        self.yaw = wrap(self.yaw + yaw_rate * self.dt)
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        return yaw_rate

    def _tick(self):
        if self.paused:
            yaw_rate = 0.0
        elif self.track == "oval":
            yaw_rate = self._tick_oval()
        else:
            yaw_rate = self._tick_grid()
        if not self.paused:
            self.wheel_spin += (self.v / self.rear_r) * self.dt
        # visual lean into the turn (coordinated-turn roll), purely cosmetic
        roll = math.atan2(self.v * yaw_rate, G)

        now = self.get_clock().now().to_msg()
        q = self._quat(roll, 0.0, self.yaw)

        tfm = TransformStamped()
        tfm.header.stamp = now
        tfm.header.frame_id = "odom"
        tfm.child_frame_id = "base_link"
        tfm.transform.translation.x = self.x
        tfm.transform.translation.y = self.y
        tfm.transform.translation.z = self.rear_r
        tfm.transform.rotation = q
        self.tf.sendTransform(tfm)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.position.z = self.rear_r
        od.pose.pose.orientation = q
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = yaw_rate
        self.pub.publish(od)

    @staticmethod
    def _quat(roll, pitch, yaw):
        from geometry_msgs.msg import Quaternion
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args=None):
    rclpy.init(args=args)
    node = PathDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
