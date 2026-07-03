"""city_streamer.py — endless city: spawn/delete block tiles around the bike.

Subscribes /moto/odom and keeps a (2W+1)×(2W+1) window of city block tiles spawned
around the bike's current grid cell, via Gazebo's /spawn_entity and /delete_entity
services. Tiles are deterministic per (i,j), so revisiting a cell looks identical.
The bike can ride forever and the city is always there ahead of it.

Pairs with worlds.generate_empty_world() (giant ground only) — run this instead of
the fixed city world.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

from moto_gazebo.city_world import tile_model_sdf, TILE_SIZE


class CityStreamer(Node):
    def __init__(self):
        super().__init__("city_streamer")
        self.declare_parameter("tile_size", TILE_SIZE)
        self.declare_parameter("window", 2)          # radius in tiles → 5×5
        self.T = float(self.get_parameter("tile_size").value)
        self.W = int(self.get_parameter("window").value)

        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_cli = self.create_client(DeleteEntity, "/delete_entity")
        self.spawned: set[tuple[int, int]] = set()
        self.pending: set[tuple[int, int]] = set()
        self._bike = (0.0, 0.0)
        self._last_cell = None

        self.create_subscription(Odometry, "/moto/odom", self._odom, 10)
        self.create_timer(0.5, self._update)   # re-evaluate window at 2 Hz
        self.get_logger().info(f"city_streamer: tile {self.T} m, window {self.W} (waiting for services + odom)")

    def _odom(self, m: Odometry):
        self._bike = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _cell(self, x, y):
        return (math.floor(x / self.T), math.floor(y / self.T))

    def _update(self):
        if not (self.spawn_cli.service_is_ready() and self.delete_cli.service_is_ready()):
            self.get_logger().warn("spawn/delete services not ready (is the world up?)",
                                   throttle_duration_sec=5.0)
            return
        bi, bj = self._cell(*self._bike)
        if (bi, bj) == self._last_cell:
            return
        self._last_cell = (bi, bj)

        desired = {(i, j) for i in range(bi - self.W, bi + self.W + 1)
                   for j in range(bj - self.W, bj + self.W + 1)}
        # spawn newcomers
        for (i, j) in desired - self.spawned - self.pending:
            self._spawn(i, j)
        # delete stragglers
        for (i, j) in self.spawned - desired:
            self._delete(i, j)

    def _spawn(self, i, j):
        req = SpawnEntity.Request()
        req.name = f"tile_{i}_{j}"
        req.xml = tile_model_sdf(i, j, self.T)
        req.reference_frame = "world"
        self.pending.add((i, j))

        def done(fut, key=(i, j)):
            self.pending.discard(key)
            try:
                if fut.result() is not None and fut.result().success:
                    self.spawned.add(key)
            except Exception:
                pass
        self.spawn_cli.call_async(req).add_done_callback(done)

    def _delete(self, i, j):
        req = DeleteEntity.Request()
        req.name = f"tile_{i}_{j}"
        self.spawned.discard((i, j))
        self.delete_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = CityStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
