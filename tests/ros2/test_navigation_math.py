import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws"
    / "src"
    / "amr_controller"
)
sys.path.insert(0, str(PACKAGE_ROOT))

from amr_controller.navigation_math import (  # noqa: E402
    calculate_velocity_command,
    normalize_angle,
)


DEFAULTS = {
    "linear_gain": 0.65,
    "angular_gain": 1.4,
    "min_linear_speed": 0.15,
    "max_linear_speed": 0.75,
    "max_angular_speed": 0.55,
    "distance_tolerance": 0.15,
}


class NavigationMathTest(unittest.TestCase):
    def test_angle_is_normalized(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi)

    def test_goal_in_front_drives_forward(self):
        command = calculate_velocity_command(
            0.0, 0.0, 0.0, 3.0, 0.0, **DEFAULTS
        )
        self.assertGreater(command.linear, 0.0)
        self.assertAlmostEqual(command.angular, 0.0)
        self.assertFalse(command.reversing)

    def test_goal_to_left_steers_left(self):
        command = calculate_velocity_command(
            0.0, 0.0, 0.0, 2.0, 2.0, **DEFAULTS
        )
        self.assertGreater(command.linear, 0.0)
        self.assertGreater(command.angular, 0.0)

    def test_goal_behind_selects_reverse(self):
        command = calculate_velocity_command(
            0.0, 0.0, 0.0, -2.0, 0.0, **DEFAULTS
        )
        self.assertLess(command.linear, 0.0)
        self.assertAlmostEqual(command.angular, 0.0)
        self.assertTrue(command.reversing)

    def test_close_goal_stops(self):
        command = calculate_velocity_command(
            0.0, 0.0, 0.0, 0.1, 0.0, **DEFAULTS
        )
        self.assertEqual(command.linear, 0.0)
        self.assertEqual(command.angular, 0.0)
        self.assertTrue(command.reached)

    def test_speed_is_limited(self):
        command = calculate_velocity_command(
            0.0, 0.0, 0.0, 100.0, 0.0, **DEFAULTS
        )
        self.assertLessEqual(abs(command.linear), 0.75)
        self.assertLessEqual(abs(command.angular), 0.55)


if __name__ == "__main__":
    unittest.main()
