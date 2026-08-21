from pathlib import Path
import sys
import unittest


ISAAC_ROOT = Path(__file__).resolve().parents[2] / "isaac_sim"
sys.path.insert(0, str(ISAAC_ROOT))

from cargo.container_policy import should_release_payload  # noqa: E402


class ContainerPolicyTest(unittest.TestCase):
    def test_releases_only_when_lifted_lowered_and_at_goal(self):
        self.assertTrue(should_release_payload(0.20, True, 0.02))
        self.assertFalse(should_release_payload(0.20, False, 0.02))
        self.assertFalse(should_release_payload(0.20, True, 0.20))
        self.assertFalse(should_release_payload(0.50, True, 0.02))

    def test_invalid_tolerances_raise(self):
        with self.assertRaises(ValueError):
            should_release_payload(0.0, True, 0.0, goal_tolerance=0.0)
        with self.assertRaises(ValueError):
            should_release_payload(0.0, True, 0.0, lowered_tolerance=-0.1)


if __name__ == "__main__":
    unittest.main()

