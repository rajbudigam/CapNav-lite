import json
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry


@dataclass
class CalibStats:
    n_samples: int = 0
    turn_effort: float = 0.0
    reverse_fraction: float = 0.0
    action_noise: float = 0.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ProfileCalibrator(Node):
    '''
    Skeleton calibrator.

    Design goal:
    - finishes in about 3 minutes
    - produces interpretable parameters with confidence bounds
    - avoids heavy compute

    This scaffold estimates:
    - turning_cost: proxy derived from turning effort during a short turn task
    - backtracking_cost: proxy derived from reverse fraction during a short back-up task
    - slip: proxy derived from joystick command jitter

    The v0.1 implementation should replace these proxies with the final estimation logic
    used by the tabular learner. For example, fit a small likelihood model of action
    execution vs intent and bootstrap confidence bounds over episodes.
    '''

    def __init__(self):
        super().__init__('capnav_lite_profile_calibrator')
        self.declare_parameter('duration_s', 180.0)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_path', 'profile.json')
        self.declare_parameter('user_id', 'anon_user')

        self.duration_s = float(self.get_parameter('duration_s').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.output_path = str(self.get_parameter('output_path').value)
        self.user_id = str(self.get_parameter('user_id').value)

        self.stats = CalibStats()
        self._last_axes = None

        self.create_subscription(Joy, self.joy_topic, self.on_joy, 50)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)

        self.start_time = time.time()
        self.timer = self.create_timer(0.5, self.on_tick)

        self.get_logger().info(
            f'Calibration started for {self.duration_s:.0f}s. Output: {self.output_path}'
        )

    def on_joy(self, msg: Joy):
        if len(msg.axes) < 2:
            return
        x = float(msg.axes[0])
        y = float(msg.axes[1])

        self.stats.n_samples += 1

        if self._last_axes is not None:
            dx = x - self._last_axes[0]
            dy = y - self._last_axes[1]
            self.stats.action_noise += (dx * dx + dy * dy)
        self._last_axes = (x, y)

        if y < -0.2:
            self.stats.reverse_fraction += 1.0

        self.stats.turn_effort += abs(x)

    def on_odom(self, msg: Odometry):
        wz = float(msg.twist.twist.angular.z)
        self.stats.turn_effort += 0.2 * abs(wz)

    def on_tick(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration_s:
            self.finish()

    def finish(self):
        n = max(self.stats.n_samples, 1)

        reverse_frac = self.stats.reverse_fraction / n
        turn_effort = self.stats.turn_effort / n
        noise = self.stats.action_noise / max(n - 1, 1)

        turning_cost = clamp(10.0 * turn_effort, 0.0, 10.0)
        backtracking_cost = clamp(10.0 * reverse_frac, 0.0, 10.0)
        slip = clamp(noise, 0.0, 1.0)

        tc_ci = [clamp(turning_cost - 0.5, 0.0, 10.0), clamp(turning_cost + 0.5, 0.0, 10.0)]
        bc_ci = [clamp(backtracking_cost - 0.5, 0.0, 10.0), clamp(backtracking_cost + 0.5, 0.0, 10.0)]
        slip_ci = [clamp(slip - 0.05, 0.0, 1.0), clamp(slip + 0.05, 0.0, 1.0)]

        profile = {
            'profile_version': '0.1',
            'user_id': self.user_id,
            'created_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'capability': {
                'turning_cost': round(turning_cost, 3),
                'backtracking_cost': round(backtracking_cost, 3),
                'slip': round(slip, 3),
            },
            'confidence': {
                'turning_cost_ci95': [round(tc_ci[0], 3), round(tc_ci[1], 3)],
                'backtracking_cost_ci95': [round(bc_ci[0], 3), round(bc_ci[1], 3)],
                'slip_ci95': [round(slip_ci[0], 3), round(slip_ci[1], 3)],
                'n_episodes': 60,
            },
            'safety_limits': {
                'max_omega_rad_s': 0.6,
                'max_v_m_s': 0.35,
                'max_turns_per_meter': 0.9,
                'min_clearance_m': 0.25,
                'fallback_policy': 'shared_control',
            },
            'notes': 'Skeleton calibration. Replace proxy estimation with final model in v0.1.',
        }

        out = Path(self.output_path)
        out.write_text(json.dumps(profile, indent=2), encoding='utf-8')
        self.get_logger().info(f'Wrote profile to {str(out.resolve())}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ProfileCalibrator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
