"""Check the reset acknowledgement barrier without ROS or Isaac Sim installed."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1]


def load_adapter_methods():
    path = ROOT / "scripts/process_mqtt_adapter.py"
    tree = ast.parse(path.read_text())
    original = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "Ros2ProcessMqttAdapter"
    )
    names = {
        "_on_system_result", "_on_mission_reset_ack",
        "_finish_reset_if_ready", "_publish_system_result",
    }
    selected = ast.ClassDef(
        name=original.name, bases=[], keywords=[], decorator_list=[],
        body=[node for node in original.body
              if isinstance(node, ast.FunctionDef) and node.name in names],
    )
    module = ast.fix_missing_locations(ast.Module(body=[selected], type_ignores=[]))
    namespace = {"json": json, "MQTT_SYSTEM_RESULT_TOPIC": "controltower/system/command/result"}
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[original.name]


def load_mission_methods():
    path = ROOT / "ros2_ws/src/amr_controller/amr_controller/amr_p3020_mission.py"
    tree = ast.parse(path.read_text())
    original = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "AmrP3020Mission"
    )
    names = {"_system_command_callback", "_mission_start_callback"}
    selected = ast.ClassDef(
        name=original.name, bases=[], keywords=[], decorator_list=[],
        body=[node for node in original.body
              if isinstance(node, ast.FunctionDef) and node.name in names],
    )
    module = ast.fix_missing_locations(ast.Module(body=[selected], type_ignores=[]))
    namespace = {"json": json, "String": SimpleNamespace}
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[original.name]


class SystemResetProtocolTest(unittest.TestCase):
    def test_reset_waits_for_both_isaac_and_mission(self):
        adapter = load_adapter_methods()()
        adapter.pending_reset = {
            "reset-1": {"isaac_result": None, "mission_ack": False}
        }
        adapter.mqtt_client = Mock()
        adapter.last_process_state = {"amr": "LIFTING"}
        adapter.pending_commands = {"SORTER_A": {"action": "START"}}
        adapter.pending_arrival_command = {"command_id": 1}

        result = {
            "request_id": "reset-1", "command": "RESET", "status": "SUCCESS",
        }
        adapter._on_system_result(SimpleNamespace(data=json.dumps(result)))
        adapter.mqtt_client.publish.assert_not_called()

        adapter._on_mission_reset_ack(SimpleNamespace(data="reset-1"))
        adapter.mqtt_client.publish.assert_called_once()
        self.assertEqual(adapter.last_process_state, {})
        self.assertEqual(adapter.pending_commands, {})
        self.assertIsNone(adapter.pending_arrival_command)
        self.assertNotIn("reset-1", adapter.pending_reset)

    def test_isaac_failure_is_reported_without_mission_ack(self):
        adapter = load_adapter_methods()()
        adapter.pending_reset = {
            "reset-2": {"isaac_result": None, "mission_ack": False}
        }
        adapter.mqtt_client = Mock()
        adapter._on_system_result(SimpleNamespace(data=json.dumps({
            "request_id": "reset-2", "command": "RESET", "status": "FAILED",
            "error": "world reset failed",
        })))
        payload = json.loads(adapter.mqtt_client.publish.call_args.args[1])
        self.assertEqual(payload["error"], "world reset failed")
        self.assertNotIn("reset-2", adapter.pending_reset)

    def test_mission_reset_cancels_goals_and_waits_for_create(self):
        mission = load_mission_methods()()
        mission.state = "P3020_WORKING"
        mission._generation = 0
        mission._last_reset_request_id = None
        mission.nav_goal_handle = Mock()
        mission.p3020_goal_handle = Mock()
        mission.goal_in_flight = True
        mission.current_nav_purpose = "DELIVERY"
        mission.pickup_state = "PICKUP_DONE"
        mission.reset_ack_pub = Mock()
        mission._publish_pickup_command = Mock()
        mission._set_state = Mock(side_effect=lambda state: setattr(mission, "state", state))

        reset = SimpleNamespace(data=json.dumps({"command": "RESET", "request_id": "reset-1"}))
        mission._system_command_callback(reset)
        self.assertEqual(mission.state, "WAIT_MISSION_START")
        self.assertEqual(mission._generation, 1)
        self.assertFalse(mission.goal_in_flight)
        self.assertEqual(mission.reset_ack_pub.publish.call_args.args[0].data, "reset-1")
        mission._publish_pickup_command.assert_called_once_with("RESET")

        mission._system_command_callback(reset)
        self.assertEqual(mission._generation, 1)
        mission._mission_start_callback(SimpleNamespace(data="MISSION-1"))
        self.assertEqual(mission.state, "WAIT_READY")


class PoseResetTimingTest(unittest.TestCase):
    def test_camera_reset_restarts_acquisition_without_reinitializing(self):
        path = ROOT / "isaac_sim/robots/p3020/vision/camera.py"
        tree = ast.parse(path.read_text())
        original = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                        and n.name == "CameraInterface")
        method = next(n for n in original.body if isinstance(n, ast.FunctionDef)
                      and n.name == "post_reset")
        selected = ast.ClassDef(name=original.name, bases=[], keywords=[],
                               decorator_list=[], body=[method])
        namespace = {}
        module = ast.fix_missing_locations(ast.Module(body=[selected], type_ignores=[]))
        exec(compile(module, str(path), "exec"), namespace)
        wrapper = namespace[original.name]()
        sensor = Mock()
        sensor._previous_time = 100.0
        sensor._elapsed_time = -99.0
        def reset_timing():
            sensor._previous_time = None
            sensor._elapsed_time = 0.0
        sensor.post_reset.side_effect = reset_timing
        wrapper._camera = sensor
        wrapper.post_reset()
        self.assertIsNone(sensor._previous_time)
        self.assertEqual(sensor._elapsed_time, 0.0)
        sensor.resume.assert_called_once()
        sensor.initialize.assert_not_called()

    def test_robot_and_project_clocks_keep_time_after_stop(self):
        path = ROOT / "isaac_sim/main_mission.py"
        tree = ast.parse(path.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == "_preserve_ros_time_on_stop")
        nodes = []
        for kind, flag in (
            ("isaacsim.core.nodes.IsaacReadSimulationTime", "inputs:resetOnStop"),
            ("isaacsim.ros2.bridge.ROS2RtxLidarHelper", "inputs:resetSimulationTimeOnStop"),
        ):
            attributes = {"node:type": Mock(), flag: Mock()}
            attributes["node:type"].Get.return_value = kind
            node = Mock()
            node.GetAttribute.side_effect = lambda name, attrs=attributes: attrs.get(name)
            nodes.append((node, attributes[flag]))
        stage = Mock()
        stage.Traverse.return_value = [node for node, _ in nodes]
        omni = Mock()
        omni.usd.get_context.return_value.get_stage.return_value = stage
        namespace = {"omni": omni}
        module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
        exec(compile(module, str(path), "exec"), namespace)
        namespace[function.name]()
        for _, attribute in nodes:
            attribute.Set.assert_called_once_with(False)

    def test_pose_initialization_waits_for_matching_reset_result(self):
        path = ROOT / "scripts/pose_sync_manager.py"
        tree = ast.parse(path.read_text())
        original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PoseSyncManager")
        selected = ast.ClassDef(
            name=original.name, bases=[], keywords=[], decorator_list=[],
            body=[n for n in original.body if isinstance(n, ast.FunctionDef)
                  and n.name in {"system_command_callback", "system_result_callback"}],
        )
        import queue
        import time
        import uuid
        namespace = dict(json=json, queue=queue, time=time, uuid=uuid,
                         String=SimpleNamespace, RESTORE_READY="READY", RESTORE_FAILED="FAILED")
        module = ast.fix_missing_locations(ast.Module(body=[selected], type_ignores=[]))
        exec(compile(module, str(path), "exec"), namespace)
        node = namespace[original.name]()
        node.last_system_reset_id = None
        node.get_logger = Mock()
        node.system_command_callback(SimpleNamespace(data=json.dumps({
            "command": "RESET", "request_id": "r1",
        })))
        self.assertEqual(node.restore_state, "RESETTING")
        node.latest_pose = (9, -6, 0)  # A sample from before world.reset().
        node.system_result_callback(SimpleNamespace(data=json.dumps({
            "command": "RESET", "request_id": "old", "status": "SUCCESS",
        })))
        self.assertEqual(node.restore_state, "RESETTING")
        node.system_result_callback(SimpleNamespace(data=json.dumps({
            "command": "RESET", "request_id": "r1", "status": "SUCCESS",
        })))
        self.assertEqual(node.restore_state, "READY")
        self.assertIsNone(node.latest_pose)
        self.assertTrue(node.nav2_needs_initial_pose)


if __name__ == "__main__":
    unittest.main()
