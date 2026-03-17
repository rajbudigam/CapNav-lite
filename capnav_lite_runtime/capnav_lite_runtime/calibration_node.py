from __future__ import annotations

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from capnav_lite_core.calibration import load_trial_csv, estimate_profile_from_trials
from capnav_lite_core.profile import save_profile


class CalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("capnav_lite_calibration")
        self.declare_parameter("input_csv", "")
        self.declare_parameter("output_profile", "profiles/calibrated_user.json")
        self.publisher = self.create_publisher(String, "calibration_status", 10)
        self.create_timer(1.0, self._run_once)
        self.done = False

    def _run_once(self) -> None:
        if self.done:
            return
        csv_path = self.get_parameter("input_csv").value
        output_profile = self.get_parameter("output_profile").value
        msg = String()
        try:
            points = load_trial_csv(csv_path)
            profile, summary = estimate_profile_from_trials(points, user_id="calibrated-user")
            save_profile(profile, output_profile)
            msg.data = f"saved {output_profile} with slip={summary.inferred_slip_probability:.3f} turning_cost={summary.inferred_turning_cost:.3f}"
        except Exception as exc:
            msg.data = f"calibration failed: {exc}"
        self.publisher.publish(msg)
        self.get_logger().info(msg.data)
        self.done = True


def main() -> None:
    rclpy.init()
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
