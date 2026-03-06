import json
import math
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class SafetyShell(Node):
    '''
    Runtime safety shell for CapNav-Lite.

    Responsibilities:
    - enforce conservative velocity and turning limits
    - enforce minimum clearance using LaserScan (or other proximity sensor)
    - publish stop commands when a rule is violated
    - emit human-readable alerts

    The v0.1 build should integrate with the actual platform:
    - E-stop hardware line
    - command mux (Nav2 vs joystick) for shared-control fallback
    '''

    def __init__(self):
        super().__init__('capnav_lite_safety_shell')
        self.declare_parameter('profile_path', '')
        self.declare_parameter('cmd_in', '/cmd_vel')
        self.declare_parameter('cmd_out', '/cmd_vel_safe')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('alerts_topic', '/capnav_lite/alerts')

        self.profile_path = str(self.get_parameter('profile_path').value)
        self.cmd_in = str(self.get_parameter('cmd_in').value)
        self.cmd_out = str(self.get_parameter('cmd_out').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.alerts_topic = str(self.get_parameter('alerts_topic').value)

        self.max_v = 0.35
        self.max_omega = 0.6
        self.max_turns_per_meter = 0.9
        self.min_clearance = 0.25
        self.fallback_policy = 'shared_control'

        self._load_profile()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_out, 10)
        self.alert_pub = self.create_publisher(String, self.alerts_topic, 10)

        self.create_subscription(Twist, self.cmd_in, self.on_cmd, 50)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.last_scan_min = None
        self.integrated_turn = 0.0
        self.integrated_dist = 0.0
        self.last_time = time.time()

        self.get_logger().info(
            f'Safety shell active. cmd_in={self.cmd_in} cmd_out={self.cmd_out} scan={self.scan_topic}'
        )

    def _load_profile(self):
        if not self.profile_path:
            self.get_logger().warn('No profile_path set, using conservative defaults.')
            return
        try:
            data = json.loads(Path(self.profile_path).read_text(encoding='utf-8'))
            limits = data.get('safety_limits', {})
            self.max_v = float(limits.get('max_v_m_s', self.max_v))
            self.max_omega = float(limits.get('max_omega_rad_s', self.max_omega))
            self.max_turns_per_meter = float(limits.get('max_turns_per_meter', self.max_turns_per_meter))
            self.min_clearance = float(limits.get('min_clearance_m', self.min_clearance))
            self.fallback_policy = str(limits.get('fallback_policy', self.fallback_policy))
        except Exception as e:
            self.get_logger().warn(f'Could not load profile: {e}. Using defaults.')

    def on_scan(self, msg: LaserScan):
        # Keep a robust minimum distance estimate
        vals = [r for r in msg.ranges if math.isfinite(r)]
        self.last_scan_min = min(vals) if vals else None

    def _alert(self, text: str):
        m = String()
        m.data = text
        self.alert_pub.publish(m)
        self.get_logger().warn(text)

    def _publish_stop(self, reason: str):
        self._alert(f'SAFETY_STOP: {reason} | policy={self.fallback_policy}')
        z = Twist()
        self.cmd_pub.publish(z)

    def on_cmd(self, msg: Twist):
        now = time.time()
        dt = max(now - self.last_time, 1e-3)
        self.last_time = now

        v = float(msg.linear.x)
        omega = float(msg.angular.z)

        # Track turning per meter: integral |omega| dt / integral |v| dt
        self.integrated_turn += abs(omega) * dt
        self.integrated_dist += abs(v) * dt

        turns_per_meter = self.integrated_turn / max(self.integrated_dist, 1e-3)

        # Reset integrals every few seconds to keep it local
        if self.integrated_dist > 2.0:
            self.integrated_turn *= 0.5
            self.integrated_dist *= 0.5

        if abs(v) > self.max_v + 1e-6:
            return self._publish_stop(f'velocity_limit_exceeded v={v:.3f} max_v={self.max_v:.3f}')

        if abs(omega) > self.max_omega + 1e-6:
            return self._publish_stop(f'omega_limit_exceeded w={omega:.3f} max_w={self.max_omega:.3f}')

        if turns_per_meter > self.max_turns_per_meter + 1e-6:
            return self._publish_stop(f'turn_density_exceeded tpm={turns_per_meter:.3f} max={self.max_turns_per_meter:.3f}')

        if self.last_scan_min is not None and self.last_scan_min < self.min_clearance:
            return self._publish_stop(f'clearance_violation dmin={self.last_scan_min:.3f} min={self.min_clearance:.3f}')

        # If safe, forward command (with hard clamp)
        out = Twist()
        out.linear.x = clamp(v, -self.max_v, self.max_v)
        out.angular.z = clamp(omega, -self.max_omega, self.max_omega)
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyShell()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
