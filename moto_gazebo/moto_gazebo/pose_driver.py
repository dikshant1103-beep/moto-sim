"""pose_driver.py — drive the Gazebo sensor_bike from Chrono telemetry (A3 core).

Subscribes to /moto/odom (published by moto_ros ws_bridge, which mirrors the
PyChrono physics) and teleports the kinematic `sensor_bike` model to that pose in
Gazebo every cycle via the /moto/set_entity_state service. Gazebo then renders the
onboard camera + lidar from the real ride trajectory and publishes them as ROS 2
topics → SLAM.

This is the clean split the user chose: **Chrono = physics/balancing, Gazebo =
sensor world (render only)**. No bike physics is solved in Gazebo, so there's no
balancing problem here.

It also broadcasts TF map→odom (identity) so camera/lidar frames chain to a SLAM map.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


class PoseDriver(Node):
    def __init__(self):
        super().__init__("moto_pose_driver")
        self.declare_parameter("entity", "sensor_bike")
        self.declare_parameter("odom_topic", "/moto/odom")
        self.declare_parameter("state_service", "/moto/set_entity_state")
        self.declare_parameter("rate_hz", 50.0)
        self.entity = self.get_parameter("entity").value
        odom_topic = self.get_parameter("odom_topic").value
        state_service = self.get_parameter("state_service").value

        self.cli = self.create_client(SetEntityState, state_service)
        self._last = None
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._drive)
        self._warned = False
        self.get_logger().info(f"pose_driver: posing '{self.entity}' from {odom_topic}")

    def _odom_cb(self, msg: Odometry):
        self._last = msg

    def _drive(self):
        if self._last is None:
            return
        if not self.cli.service_is_ready():
            if not self._warned:
                self.get_logger().warn("/moto/set_entity_state not ready — is the city world up?",
                                       throttle_duration_sec=5.0)
                self._warned = True
            return

        st = EntityState()
        st.name = self.entity
        st.reference_frame = "world"
        st.pose = self._last.pose.pose
        st.twist = self._last.twist.twist

        req = SetEntityState.Request()
        req.state = st
        self.cli.call_async(req)  # fire-and-forget; next frame supersedes


def main(args=None):
    rclpy.init(args=args)
    node = PoseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
