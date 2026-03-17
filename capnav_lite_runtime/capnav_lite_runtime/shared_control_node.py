from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from capnav_lite_core import CapabilityProfile, SafetyEnvelope, VelocityCommand, load_profile


class SharedControlNode(Node):
    def __init__(self) -> None:
        super().__init__("capnav_lite_shared_control")
        self.declare_parameter("profile_path", "")
        profile_path = self.get_parameter("profile_path").value
        profile = load_profile(profile_path) if profile_path else CapabilityProfile()
        self.safety = SafetyEnvelope(profile)
        self.latest_user = Twist()
        self.latest_autonomy = Twist()
        self.latest_risk = 0.2
        self.latest_confidence = 0.5
        self.create_subscription(Twist, "user_cmd_vel", self._user_cb, 10)
        self.create_subscription(Twist, "safe_cmd_vel", self._autonomy_cb, 10)
        self.create_subscription(Float32, "navigation_risk", self._risk_cb, 10)
        self.create_subscription(Float32, "autonomy_confidence", self._confidence_cb, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.05, self._tick)

    def _user_cb(self, msg: Twist) -> None:
        self.latest_user = msg

    def _autonomy_cb(self, msg: Twist) -> None:
        self.latest_autonomy = msg

    def _risk_cb(self, msg: Float32) -> None:
        self.latest_risk = msg.data

    def _confidence_cb(self, msg: Float32) -> None:
        self.latest_confidence = msg.data

    def _tick(self) -> None:
        decision = self.safety.blend(
            VelocityCommand(self.latest_autonomy.linear.x, self.latest_autonomy.angular.z, 0.0, "AUTO"),
            VelocityCommand(self.latest_user.linear.x, self.latest_user.angular.z, 0.0, "USER"),
            risk_score=self.latest_risk,
            autonomy_confidence=self.latest_confidence,
        )
        out = Twist()
        out.linear.x = decision.safe_command.linear_mps
        out.angular.z = decision.safe_command.angular_rad_s
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = SharedControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
