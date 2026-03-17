from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from capnav_lite_core import CapabilityProfile, SafetyEnvelope, VelocityCommand, load_profile


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("capnav_lite_safety_monitor")
        self.declare_parameter("profile_path", "")
        profile_path = self.get_parameter("profile_path").value
        profile = load_profile(profile_path) if profile_path else CapabilityProfile()
        self.safety = SafetyEnvelope(profile)
        self.latest_scan: LaserScan | None = None

        self.create_subscription(Twist, "planner_cmd_vel", self._cmd_callback, 10)
        self.create_subscription(LaserScan, "scan", self._scan_callback, 10)
        self.publisher = self.create_publisher(Twist, "safe_cmd_vel", 10)

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _cmd_callback(self, msg: Twist) -> None:
        if self.latest_scan is None:
            return
        valid_ranges = [r for r in self.latest_scan.ranges if math.isfinite(r) and r > self.latest_scan.range_min]
        front_clearance = min(valid_ranges) if valid_ranges else self.latest_scan.range_max
        linear_speed = abs(msg.linear.x)
        ttc = front_clearance / max(0.05, linear_speed) if linear_speed > 0 else 99.0
        safe = self.safety.enforce(
            VelocityCommand(msg.linear.x, msg.angular.z, 0.0, "PLANNER"),
            front_clearance_m=front_clearance,
            estimated_ttc_s=ttc,
            risk_score=0.2 if ttc > 2.0 else 0.9,
        )
        out = Twist()
        out.linear.x = safe.linear_mps
        out.angular.z = safe.angular_rad_s
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
