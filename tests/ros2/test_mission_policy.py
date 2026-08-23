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

from amr_controller.mission_policy import linear_ramp  # noqa: E402


class MissionPolicyTest(unittest.TestCase):
    def test_lift_ramp_midpoint(self):
        value, complete = linear_ramp(0.0, 0.30, 2.0, 4.0)
        self.assertAlmostEqual(value, 0.15)
        self.assertFalse(complete)

    def test_ramp_clamps_before_and_after_duration(self):
        self.assertEqual(linear_ramp(0.0, 0.30, -1.0, 4.0), (0.0, False))
        self.assertEqual(linear_ramp(0.30, 0.0, 5.0, 4.0), (0.0, True))

    def test_invalid_duration_raises(self):
        with self.assertRaises(ValueError):
            linear_ramp(0.0, 0.30, 1.0, 0.0)


if __name__ == "__main__":
    unittest.main()

