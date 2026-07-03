"""roam_goals.py — feed nav2 a stream of roaming goals along the road grid.

Instead of dead-reckoning steering, the bike now roams by sending NavigateToPose
goals to nav2, which plans a collision-free path (avoiding buildings seen by the
lidar) and the cmd_vel_to_moto adapter makes the bike follow. On reaching/aborting
a goal, a new one is chosen: the next road intersection a couple of blocks ahead in
a randomly chosen direction (biased to keep going forward). This makes the bike
explore the endless city autonomously while genuinely avoiding obstacles.
"""

from __future__ import annotations

import math
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from moto_gazebo.city_world import TILE_SIZE

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


class RoamGoals(Node):
    def __init__(self):
        super().__init__("roam_goals")
        self.declare_parameter("tile_size", TILE_SIZE)
        self.declare_parameter("hops", 2)            # intersections ahead per goal
        self.declare_parameter("straight_bias", 0.6)
        self.T = float(self.get_parameter("tile_size").value)
        self.hops = int(self.get_parameter("hops").value)
        self.bias = float(self.get_parameter("straight_bias").value)

        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self._dir = (1, 0)
        self._busy = False

        self.create_subscription(Odometry, "/moto/odom", self._odom, 10)
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(1.0, self._tick)
        self.get_logger().info("roam_goals: waiting for nav2 + odom, then sending roaming goals")

    def _odom(self, m: Odometry):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def _tick(self):
        if self._busy or not self.have_odom:
            return
        if not self.ac.server_is_ready():
            self.get_logger().warn("nav2 navigate_to_pose action not ready…", throttle_duration_sec=5.0)
            return
        self._send_next()

    def _nearest_intersection(self):
        return (round(self.x / self.T) * self.T, round(self.y / self.T) * self.T)

    def _choose_dir(self):
        back = (-self._dir[0], -self._dir[1])
        choices = [d for d in DIRS if d != back]
        weights = [self.bias if d == self._dir else (1 - self.bias) / 2 for d in choices]
        self._dir = random.choices(choices, weights=weights)[0]
        return self._dir

    def _send_next(self):
        gx, gy = self._nearest_intersection()
        d = self._choose_dir()
        gx += d[0] * self.T * self.hops
        gy += d[1] * self.T * self.hops
        yaw = math.atan2(d[1], d[0])

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.orientation.z = math.sin(yaw / 2)
        ps.pose.orientation.w = math.cos(yaw / 2)
        goal.pose = ps

        self._busy = True
        self.get_logger().info(f"goal → ({gx:.0f}, {gy:.0f})")
        self.ac.send_goal_async(goal).add_done_callback(self._accepted)

    def _accepted(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().warn("goal rejected; retrying soon")
            self._busy = False
            return
        gh.get_result_async().add_done_callback(lambda f: self._done())

    def _done(self):
        # reached / aborted / canceled — pick the next goal next tick
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = RoamGoals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
