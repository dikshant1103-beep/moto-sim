"""roam_controller.py — autonomous roaming on the endless city's road grid.

Drives the PyChrono bike around the city by itself: heads to the next road
intersection, and at each intersection randomly continues / turns left / turns
right (never reverses). Steers the self-balancing bike by commanding rider LEAN
(moto_sim leans into the turn → steady turn) over the sim's WS command channel.
Speed is held by the sim's own speed controller, so we only steer.

Roads run along the grid lines x=k·T and y=k·T (the city_streamer tile edges);
intersections are at (k·T, m·T). T must match the streamer's tile size.

Sends to moto_sim (ws://localhost:8765):  {"cmd":"set","key":"lean_delta","value":θ}

NOTE: the lean→turn SIGN is bike-convention dependent; if it steers the wrong way,
flip `lean_sign` (param). Gains likely need light on-machine tuning.
"""

from __future__ import annotations

import json
import math
import random
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

try:
    from websockets.sync.client import connect as ws_connect
except Exception:  # pragma: no cover
    ws_connect = None

from moto_gazebo.city_world import TILE_SIZE

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RoamController(Node):
    def __init__(self):
        super().__init__("roam_controller")
        self.declare_parameter("ws_url", "ws://localhost:8765")
        self.declare_parameter("tile_size", TILE_SIZE)
        self.declare_parameter("lean_gain", 0.35)    # rad lean per rad heading error (P)
        self.declare_parameter("yaw_damp", 0.25)     # rad lean per (rad/s) yaw rate (D) — kills wobble
        # CLEAN turning regime only: tests show lean≤0.06/steer≤0.03 corners cleanly
        # (stable speed+roll, ~76 m radius); harder commands scrub speed and get rough.
        # The model is straight-line-derived and can't do tight cornering cleanly.
        self.declare_parameter("lean_max", 0.08)     # gentle (clean regime)
        self.declare_parameter("steer_gain", 0.18)
        self.declare_parameter("steer_max", 0.04)    # gentle (clean regime)
        self.declare_parameter("ramp_rate", 0.6)     # rad/s max change in lean/steer (gentle)
        self.declare_parameter("lean_sign", 1.0)     # flip if it turns the wrong way
        self.declare_parameter("reach_radius", 20.0)  # m — large (gentle wide turns)
        self.declare_parameter("straight_bias", 0.5) # P(go straight) at an intersection
        self.ws_url = self.get_parameter("ws_url").value
        self.T = float(self.get_parameter("tile_size").value)
        self.Kp = float(self.get_parameter("lean_gain").value)
        self.Kd = float(self.get_parameter("yaw_damp").value)
        self.lean_max = float(self.get_parameter("lean_max").value)
        self.steer_gain = float(self.get_parameter("steer_gain").value)
        self.steer_max = float(self.get_parameter("steer_max").value)
        self.ramp = float(self.get_parameter("ramp_rate").value)
        self.lean_sign = float(self.get_parameter("lean_sign").value)
        self.reach = float(self.get_parameter("reach_radius").value)
        self.straight_bias = float(self.get_parameter("straight_bias").value)

        self.x = self.y = self.yaw = 0.0
        self.yaw_rate = 0.0
        self.have_odom = False
        self.lean_cmd = 0.0
        self.steer_cmd = 0.0
        # AI inputs (from the conda-python sidecars over rosbridge)
        self.yolo_avoid = 0.0       # /moto/yolo/avoid — reactive lateral steer bias
        self.qwen_decision = None   # /moto/decision — strategic intersection bias
        self._dir = (1, 0)               # current travel direction (grid)
        self._target = None              # (gx, gy) next intersection

        self.create_subscription(Odometry, "/moto/odom", self._odom, 10)
        self.create_subscription(Float64, "/moto/yolo/avoid", self._yolo, 10)
        self.create_subscription(String, "/moto/decision", self._qwen, 10)
        self.create_timer(0.05, self._control)   # 20 Hz steering

        self._stop = threading.Event()
        threading.Thread(target=self._ws_thread, daemon=True).start()
        self.get_logger().info(f"roam_controller: grid {self.T} m, leaning to steer via {self.ws_url}")

    def _odom(self, m: Odometry):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.yaw_rate = m.twist.twist.angular.z   # for heading damping
        self.have_odom = True

    def _yolo(self, m: Float64):
        self.yolo_avoid = float(m.data)           # reactive lateral steer bias

    def _qwen(self, m: String):
        d = m.data.strip().lower()
        if d in ("left", "right", "straight"):
            self.qwen_decision = d                # strategic intersection bias

    def _nearest_intersection(self):
        return (round(self.x / self.T) * self.T, round(self.y / self.T) * self.T)

    def _pick_next(self):
        """At the current intersection, choose the next direction + target.
        Qwen's high-level decision (if any) biases left/right/straight."""
        gx, gy = self._nearest_intersection()
        back = (-self._dir[0], -self._dir[1])
        choices = [d for d in DIRS if d != back]                  # never U-turn
        # base bias toward going straight
        weights = [self.straight_bias if d == self._dir else (1 - self.straight_bias) / 2
                   for d in choices]
        # Qwen strategic bias: boost the chosen relative direction
        if self.qwen_decision:
            fx, fy = self._dir                    # forward
            left = (-fy, fx); right = (fy, -fx)   # 90° left / right of forward
            want = {"straight": self._dir, "left": left, "right": right}[self.qwen_decision]
            for i, d in enumerate(choices):
                if d == want:
                    weights[i] *= 4.0             # strongly prefer Qwen's pick (if legal)
            self.qwen_decision = None             # consume once
        self._dir = random.choices(choices, weights=weights)[0]
        self._target = (gx + self._dir[0] * self.T, gy + self._dir[1] * self.T)

    def _control(self):
        if not self.have_odom:
            return
        if self._target is None:
            self._pick_next()
        gx, gy = self._target
        if math.hypot(gx - self.x, gy - self.y) < self.reach:
            self._pick_next()
            gx, gy = self._target
        bearing = math.atan2(gy - self.y, gx - self.x)
        err = wrap(bearing - self.yaw)
        # PD heading control: P drives toward the bearing, D (yaw-rate) damps the
        # left-right wobble (proportional-only overshoots and oscillates).
        ctrl = self.Kp * err - self.Kd * self.yaw_rate
        lean_t = max(-self.lean_max, min(self.lean_max, self.lean_sign * ctrl))
        # steer target — opposite sign to lean per the motor frame (this turns the bike).
        # YOLO reactive avoidance: add the vision steer-bias (steer away from obstacles
        # YOLO sees ahead). Same sign convention as steer_target (− = steer left/right).
        steer_t = (-self.lean_sign * self.steer_gain * ctrl) - self.lean_sign * self.yolo_avoid
        steer_t = max(-self.steer_max, min(self.steer_max, steer_t))
        # RATE-LIMIT both: ramp gradually so a 90° intersection turn doesn't slam the
        # steer to max in one step → that's what diverges the simplified stability model.
        dmax = self.ramp * 0.05   # per 20 Hz control tick
        self.lean_cmd += max(-dmax, min(dmax, lean_t - self.lean_cmd))
        self.steer_cmd += max(-dmax, min(dmax, steer_t - self.steer_cmd))

    # ── send lean to moto_sim over its WS command channel ────────────────────
    def _ws_thread(self):
        if ws_connect is None:
            self.get_logger().error("websockets sync client unavailable; roam idle")
            return
        while not self._stop.is_set():
            try:
                with ws_connect(self.ws_url, open_timeout=5) as ws:
                    self.get_logger().info("connected to moto_sim command channel")
                    while not self._stop.is_set():
                        ws.send(json.dumps({"cmd": "set", "key": "lean_delta",
                                            "value": round(self.lean_cmd, 4)}))
                        ws.send(json.dumps({"cmd": "set", "key": "steer_target",
                                            "value": round(self.steer_cmd, 4)}))
                        time.sleep(0.05)
            except Exception as e:
                self.get_logger().warn(f"cmd ws disconnected ({type(e).__name__}); retrying…",
                                       throttle_duration_sec=5.0)
                time.sleep(1.0)

    def destroy_node(self):
        # straighten up on exit
        self.lean_cmd = 0.0
        self.steer_cmd = 0.0
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoamController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
