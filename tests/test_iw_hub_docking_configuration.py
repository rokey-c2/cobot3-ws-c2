import ast
import math
import unittest
from pathlib import Path


class IwHubDockingConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = (
            Path(__file__).resolve().parents[1]
            / "isaac_sim"
            / "robots"
            / "iw_hub"
            / "iw_hub_mission_agent.py"
        )
        cls.source = cls.path.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    def assignment_value(self, name):
        for node in self.tree.body:
            if not isinstance(node, ast.Assign):
                continue
            if not any(
                isinstance(target, ast.Name) and target.id == name
                for target in node.targets
            ):
                continue
            if isinstance(node.value, ast.Constant):
                return node.value.value
            if (
                isinstance(node.value, ast.UnaryOp)
                and isinstance(node.value.op, ast.USub)
                and isinstance(node.value.operand, ast.Constant)
            ):
                return -node.value.operand.value
            if (
                isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Attribute)
                and node.value.func.attr == "radians"
            ):
                angle = node.value.args[0]
                if isinstance(angle, ast.Constant):
                    return math.radians(angle.value)
                if (
                    isinstance(angle, ast.UnaryOp)
                    and isinstance(angle.op, ast.USub)
                    and isinstance(angle.operand, ast.Constant)
                ):
                    return math.radians(-angle.operand.value)
        self.fail(f"assignment not found: {name}")

    def test_measured_lift_pose(self):
        self.assertEqual(self.assignment_value("TARGET_ROOT_X"), 9.0)
        self.assertEqual(self.assignment_value("TARGET_ROOT_Y"), -3.3)
        self.assertAlmostEqual(
            self.assignment_value("TARGET_YAW"),
            math.radians(-90.0),
        )

    def test_y_drive_is_reverse_at_minus_ninety_degrees(self):
        linear_assignments = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "linear"
                for target in node.targets
            )
        ]
        reverse_copysign = [
            node
            for node in linear_assignments
            if isinstance(node.value, ast.UnaryOp)
            and isinstance(node.value.op, ast.USub)
            and isinstance(node.value.operand, ast.Call)
            and isinstance(node.value.operand.func, ast.Attribute)
            and node.value.operand.func.attr == "copysign"
        ]
        self.assertEqual(len(linear_assignments), 3)
        self.assertEqual(len(reverse_copysign), 2)


if __name__ == "__main__":
    unittest.main()
