"""Exercise lift ownership and bridge feedback without Isaac's GPU runtime."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1]


def load_methods(path, name, methods, namespace):
    tree = ast.parse(path.read_text())
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    selected = ast.ClassDef(name=name, bases=[], keywords=[], decorator_list=[], body=[
        n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in methods
    ])
    module = ast.fix_missing_locations(ast.Module(body=[selected], type_ignores=[]))
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[name], tree


class ManualLiftTest(unittest.TestCase):
    def setUp(self):
        namespace = {"LIFT_TARGET": 0.04}
        cls, tree = load_methods(
            ROOT / "isaac_sim/robots/iw_hub/iw_hub_mission_agent.py",
            "MissionIwHubAgent",
            {"request_manual_lift", "get_lift_state", "on_physics_step", "_set_state"}, namespace,
        )
        constant = next(n for n in tree.body if isinstance(n, ast.Assign)
                        and any(isinstance(t, ast.Name) and t.id == "MANUAL_LIFT_STATES" for t in n.targets))
        namespace["MANUAL_LIFT_STATES"] = ast.literal_eval(constant.value)
        self.agent = cls()
        self.agent._manual_lift_active = False
        self.agent._manual_lift_target = 0.0
        self.agent._manual_lift_state = "DOWN"
        self.agent._state_elapsed = 0.0
        self.position = 0.0
        self.agent._joint_position = lambda: self.position
        self.agent._hold_lift = Mock(side_effect=self.move_lift)
        self.agent._stop = Mock()

    def move_lift(self, target):
        self.position = target

    def test_up_down_after_idle_completion_or_error(self):
        for state in ("IDLE", "LOWER_DONE", "SPAWN_DONE", "ERROR"):
            with self.subTest(state=state):
                self.agent.mission_state = state
                for action, position in (("UP", 0.04), ("DOWN", 0.0)):
                    self.assertTrue(self.agent.request_manual_lift(action))
                    self.agent.on_physics_step(0.016)
                    self.assertEqual(self.position, position)
                    self.assertEqual(self.agent.get_lift_state(), action)
                    self.assertEqual(self.agent.mission_state, state)
        self.agent._stop.assert_not_called()

    def test_active_mission_keeps_lift_ownership(self):
        for state in ("LIFTING", "PICKUP_DONE", "LOWERED_AT_DELIVERY", "RETURN_DOCK_DONE"):
            self.agent.mission_state = state
            self.assertFalse(self.agent.request_manual_lift("UP"))
            self.assertFalse(self.agent.request_manual_lift("DOWN"))
        self.assertFalse(self.agent._manual_lift_active)

    def test_error_does_not_move_lift_without_explicit_request(self):
        self.agent.mission_state = "ERROR"
        self.position = 0.02
        self.agent.on_physics_step(0.016)
        self.agent._hold_lift.assert_not_called()

    def test_new_mission_clears_previous_manual_target_ownership(self):
        self.agent.mission_state = "IDLE"
        self.assertTrue(self.agent.request_manual_lift("DOWN"))
        self.agent._set_state("ROTATE_TO_DOCK")
        self.agent._set_state("ERROR")
        self.agent.on_physics_step(0.016)
        self.agent._hold_lift.assert_not_called()

    def test_lowering_state_reports_direction_between_endpoints(self):
        self.position = 0.02
        for state in ("LOWERING", "LOWERING_AT_DELIVERY"):
            self.agent.mission_state = state
            self.assertEqual(self.agent.get_lift_state(), "MOVING_DOWN")


class LiftBridgeTest(unittest.TestCase):
    def setUp(self):
        cls, _ = load_methods(ROOT / "isaac_sim/main_mission.py", "AmrMissionBridge",
                              {"_lift_command_callback"}, {"json": json, "String": SimpleNamespace})
        self.bridge = cls()
        self.bridge.agent = Mock()
        self.bridge.agent.request_manual_lift.return_value = False
        self.bridge.agent.get_mission_state.return_value = "LIFTING"
        self.bridge.get_logger = Mock(return_value=Mock())
        self.bridge.lift_result_pub = Mock()

    def test_rejection_preserves_command_id_and_reason(self):
        self.bridge._lift_command_callback(SimpleNamespace(data=json.dumps({"command_id": 6, "action": "UP"})))
        result = json.loads(self.bridge.lift_result_pub.publish.call_args.args[0].data)
        self.assertEqual(result["command_id"], 6)
        self.assertEqual(result["status"], "REJECTED")
        self.assertIn("mission_state=LIFTING", result["error_message"])

    def test_plain_ros_command_remains_supported(self):
        self.bridge.agent.request_manual_lift.return_value = True
        self.bridge._lift_command_callback(SimpleNamespace(data=" down "))
        self.bridge.agent.request_manual_lift.assert_called_once_with("DOWN")
        self.bridge.lift_result_pub.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
