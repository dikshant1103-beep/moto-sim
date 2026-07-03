"""convoy_ghost.py — generate a ghost-bike odom from the lead bike's trajectory.

Only one PyChrono physics sim runs (the lead). To get multi-vehicle without N full
physics sims, each ghost replays the LEAD's actually-ridden path with a fixed
longitudinal lag (behind) and a lateral lane offset — a believable convoy. It
publishes /<ns>/odom, which a per-ghost pose_driver consumes to place that bike in
Gazebo (and its lidar/camera then render from there).

Honest scope: ghosts are kinematic followers of recorded real trajectory, NOT
independently simulated dynamics. Good for sensor/SLAM/multi-vehicle scene testing;
not a multi-bike interaction dynamics model.
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ConvoyGhost(Node):
    def __init__(self):
        super().__init__("convoy_ghost")
        self.declare_parameter("lead_odom", "/moto/odom")
        self.declare_parameter("ghost_odom", "/moto1/odom")
        self.declare_parameter("lag_m", 8.0)        # distance behind the lead along its path
        self.declare_parameter("lane_offset_m", 1.6)  # lateral offset (left +)
        lead = self.get_parameter("lead_odom").value
        self.ghost_topic = self.get_parameter("ghost_odom").value
        self.lag = float(self.get_parameter("lag_m").value)
        self.lane = float(self.get_parameter("lane_offset_m").value)

        # ring buffer of recent lead poses + cumulative arc length
        self._trail = deque(maxlen=4000)  # (s, x, y, yaw, vx, vy, yawrate)
        self._s = 0.0
        self._prev = None

        self.pub = self.create_publisher(Odometry, self.ghost_topic, 20)
        self.create_subscription(Odometry, lead, self._lead_cb, 20)
        self.get_logger().info(f"convoy_ghost: {lead} → {self.ghost_topic} (lag {self.lag} m, lane {self.lane} m)")

    def _lead_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        if self._prev is not None:
            self._s += math.hypot(p.x - self._prev[0], p.y - self._prev[1])
        self._prev = (p.x, p.y)
        self._trail.append((self._s, p.x, p.y, yaw,
                            msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                            msg.twist.twist.angular.z))

        # target arc position = current lead arc − lag
        target_s = self._s - self.lag
        gx, gy, gyaw, gvx, gvy, gyr = p.x, p.y, yaw, 0.0, 0.0, 0.0
        if target_s > 0 and len(self._trail) > 1:
            # find the trail sample at target_s (linear scan from oldest is fine at this size)
            for i in range(len(self._trail) - 1):
                s0 = self._trail[i][0]
                s1 = self._trail[i + 1][0]
                if s0 <= target_s <= s1:
                    t = (target_s - s0) / (s1 - s0) if s1 > s0 else 0.0
                    a, bb = self._trail[i], self._trail[i + 1]
                    gx = a[1] + t * (bb[1] - a[1])
                    gy = a[2] + t * (bb[2] - a[2])
                    gyaw = a[3] + t * (bb[3] - a[3])
                    gvx, gvy, gyr = bb[4], bb[5], bb[6]
                    break

        # apply lateral lane offset (perpendicular to heading)
        gx += -math.sin(gyaw) * self.lane
        gy += math.cos(gyaw) * self.lane

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = "base_link"
        out.pose.pose.position.x = gx
        out.pose.pose.position.y = gy
        out.pose.pose.position.z = msg.pose.pose.position.z
        cy, sy = math.cos(gyaw / 2), math.sin(gyaw / 2)
        out.pose.pose.orientation.w = cy
        out.pose.pose.orientation.z = sy
        out.twist.twist.linear.x = gvx
        out.twist.twist.linear.y = gvy
        out.twist.twist.angular.z = gyr
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConvoyGhost()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
