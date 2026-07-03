"""cmd_vel_to_moto.py — translate nav2's /cmd_vel into PyChrono motorcycle commands.

nav2 (and most planners) command a robot with geometry_msgs/Twist: linear.x (m/s)
and angular.z (rad/s yaw rate). A motorcycle can't take that directly — it turns by
LEANING and its yaw rate depends on speed. This node converts:

    desired yaw rate ω  →  lean angle θ   via the steady-turn relation
        ω = g·tan(θ) / v      ⇒   θ = atan( ω·v / g )
    desired speed v     →  the sim's speed-control target (key "v0"/"vx") + throttle

and sends them to moto_sim over its WS command channel (ws://localhost:8765):
    {"cmd":"set","key":"lean_delta","value":θ}
    {"cmd":"set","key":"target_speed","value":v}   (sim cruise-control target)

This is the adapter that lets nav2 drive the bike. The lean SIGN may need flipping
(param lean_sign) per the bike's convention; gains likely need light tuning.
"""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from websockets.sync.client import connect as ws_connect
except Exception:  # pragma: no cover
    ws_connect = None

G = 9.81


class CmdVelToMoto(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_moto")
        self.declare_parameter("ws_url", "ws://localhost:8765")
        self.declare_parameter("lean_max", 0.45)     # rad — max commanded lean
        self.declare_parameter("steer_max", 0.4)     # rad — max commanded steer-target
        self.declare_parameter("lean_sign", 1.0)     # flip if it turns the wrong way
        self.declare_parameter("wheelbase", 1.37)    # m
        self.declare_parameter("min_speed_for_lean", 1.0)  # m/s — below this, don't lean (ω→θ blows up)
        self.declare_parameter("send_speed", True)   # also command speed target
        self.ws_url = self.get_parameter("ws_url").value
        self.lean_max = float(self.get_parameter("lean_max").value)
        self.steer_max = float(self.get_parameter("steer_max").value)
        self.lean_sign = float(self.get_parameter("lean_sign").value)
        self.L = float(self.get_parameter("wheelbase").value)
        self.v_floor = float(self.get_parameter("min_speed_for_lean").value)
        self.send_speed = bool(self.get_parameter("send_speed").value)

        self.lean_cmd = 0.0
        self.steer_cmd = 0.0
        self.speed_cmd = 0.0
        self._have_cmd = False

        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        # safety: if nav2 stops publishing, straighten + coast
        self._last_cmd_t = time.time()
        self.create_timer(0.2, self._watchdog)

        self._stop = threading.Event()
        threading.Thread(target=self._ws_thread, daemon=True).start()
        self.get_logger().info(f"cmd_vel_to_moto: /cmd_vel → lean/speed via {self.ws_url}")

    def _cmd(self, m: Twist):
        v = m.linear.x
        w = m.angular.z
        v_eff = max(abs(v), self.v_floor)
        # coordinated lean for the desired yaw rate:  θ = atan(ω·v / g)
        lean = math.atan2(w * v_eff, G)
        # kinematic steer angle that actually produces that yaw rate:  δ = ω·L / v
        # SIGN: lean_delta +=lean-right, but steer_angle +=steer-LEFT (motor frame),
        # so a coordinated turn needs steer_target OPPOSITE in sign to lean.
        steer = w * self.L / v_eff
        self.lean_cmd = self.lean_sign * max(-self.lean_max, min(self.lean_max, lean))
        self.steer_cmd = -self.lean_sign * max(-self.steer_max, min(self.steer_max, steer))
        self.speed_cmd = max(0.0, v)
        self._have_cmd = True
        self._last_cmd_t = time.time()

    def _watchdog(self):
        if self._have_cmd and (time.time() - self._last_cmd_t) > 0.7:
            self.lean_cmd = 0.0    # no recent cmd → ride straight (don't keep leaning)
            self.steer_cmd = 0.0

    def _ws_thread(self):
        if ws_connect is None:
            self.get_logger().error("websockets sync client unavailable; adapter idle")
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
                        if self.send_speed and self._have_cmd:
                            ws.send(json.dumps({"cmd": "set", "key": "target_speed",
                                                "value": round(self.speed_cmd, 3)}))
                        time.sleep(0.05)
            except Exception as e:
                self.get_logger().warn(f"cmd ws disconnected ({type(e).__name__}); retrying…",
                                       throttle_duration_sec=5.0)
                time.sleep(1.0)

    def destroy_node(self):
        self.lean_cmd = 0.0
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMoto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
