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

from amr_controller.velocity_mux_policy import (  # noqa: E402
    MANUAL_SOURCE,
    NAVIGATION_SOURCE,
    STOP_SOURCE,
    select_velocity_source,
)


class VelocityMuxPolicyTest(unittest.TestCase):
    def choose(self, **overrides):
        values = {
            "emergency_stop": False,
            "navigation_enabled": True,
            "manual_age": None,
            "navigation_age": 0.1,
            "command_timeout": 0.5,
        }
        values.update(overrides)
        return select_velocity_source(**values)

    def test_emergency_stop_has_highest_priority(self):
        source = self.choose(emergency_stop=True, manual_age=0.0)
        self.assertEqual(source, STOP_SOURCE)

    def test_fresh_manual_command_overrides_navigation(self):
        source = self.choose(manual_age=0.2)
        self.assertEqual(source, MANUAL_SOURCE)

    def test_navigation_is_used_without_manual_command(self):
        self.assertEqual(self.choose(), NAVIGATION_SOURCE)

    def test_stale_commands_stop(self):
        source = self.choose(manual_age=0.6, navigation_age=0.6)
        self.assertEqual(source, STOP_SOURCE)

    def test_disabled_navigation_stops(self):
        source = self.choose(navigation_enabled=False)
        self.assertEqual(source, STOP_SOURCE)

    def test_timeout_boundary_is_still_fresh(self):
        source = self.choose(manual_age=0.5)
        self.assertEqual(source, MANUAL_SOURCE)


if __name__ == "__main__":
    unittest.main()
