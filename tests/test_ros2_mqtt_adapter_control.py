import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path


class FakeMessageInfo:
    rc = 0


class FakeMqttClient:
    def __init__(self, **kwargs):
        self.publications = []
        self.subscriptions = []

    def connect(self, *args, **kwargs):
        return 0

    def loop_start(self):
        pass

    def loop_stop(self):
        pass

    def disconnect(self):
        pass

    def subscribe(self, topic, qos=0):
        self.subscriptions.append((topic, qos))

    def publish(self, topic, payload, qos=0, retain=False):
        self.publications.append((topic, payload, qos, retain))
        return FakeMessageInfo()


class FakeLogger:
    def info(self, message):
        pass

    def warning(self, message):
        pass

    def error(self, message):
        pass


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, name):
        self.publishers = []

    def create_subscription(self, *args, **kwargs):
        return object()

    def create_publisher(self, *args, **kwargs):
        publisher = FakePublisher()
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, *args, **kwargs):
        return object()

    def get_logger(self):
        return FakeLogger()

    def destroy_node(self):
        pass


class FakeActionClient:
    def __init__(self, *args, **kwargs):
        self.sent_goals = []

    def wait_for_server(self, timeout_sec):
        return True


class FakeTwist:
    pass


class FakeString:
    def __init__(self):
        self.data = ""


class FakeNavigateToPose:
    class Goal:
        pass


class FakeGoalStatus:
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5


def install_fake_modules():
    mqtt_module = types.ModuleType("paho.mqtt.client")
    mqtt_module.Client = FakeMqttClient
    paho_module = types.ModuleType("paho")
    paho_mqtt_module = types.ModuleType("paho.mqtt")
    paho_mqtt_module.client = mqtt_module
    paho_module.mqtt = paho_mqtt_module

    rclpy_module = types.ModuleType("rclpy")
    rclpy_action_module = types.ModuleType("rclpy.action")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_qos_module = types.ModuleType("rclpy.qos")
    rclpy_action_module.ActionClient = FakeActionClient
    rclpy_node_module.Node = FakeNode
    rclpy_qos_module.qos_profile_sensor_data = object()

    action_msgs = types.ModuleType("action_msgs")
    action_msgs_msg = types.ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = FakeGoalStatus

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = FakeTwist

    nav2_msgs = types.ModuleType("nav2_msgs")
    nav2_msgs_action = types.ModuleType("nav2_msgs.action")
    nav2_msgs_action.NavigateToPose = FakeNavigateToPose

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = object

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = FakeString

    modules = {
        "paho": paho_module,
        "paho.mqtt": paho_mqtt_module,
        "paho.mqtt.client": mqtt_module,
        "rclpy": rclpy_module,
        "rclpy.action": rclpy_action_module,
        "rclpy.node": rclpy_node_module,
        "rclpy.qos": rclpy_qos_module,
        "action_msgs": action_msgs,
        "action_msgs.msg": action_msgs_msg,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav2_msgs": nav2_msgs,
        "nav2_msgs.action": nav2_msgs_action,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    sys.modules.update(modules)


install_fake_modules()
adapter_path = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "ros2_mqtt_adapter.py"
)
spec = importlib.util.spec_from_file_location("adapter_under_test", adapter_path)
adapter_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter_module)


class AdapterControlTest(unittest.TestCase):
    def setUp(self):
        self.adapter = adapter_module.Ros2MqttAdapter()

    def result_payloads(self):
        return [
            json.loads(payload)
            for topic, payload, qos, retain in self.adapter.mqtt_client.publications
            if topic == adapter_module.MQTT_COMMAND_RESULT_TOPIC
        ]

    def test_stop_blocks_navigation_and_start_enables_it_again(self):
        self.adapter.process_control_command(
            {"command_id": 101, "action": "STOP"}
        )

        self.assertFalse(self.adapter.control_enabled)
        self.assertGreaterEqual(
            len(self.adapter.stop_velocity_publisher.messages),
            1,
        )
        self.assertEqual(
            [item["status"] for item in self.result_payloads()],
            ["RUNNING", "SUCCESS"],
        )

        self.adapter.process_navigation_command(
            {"command_id": 102, "x": 1.0, "y": 2.0, "yaw": 0.0}
        )
        self.assertEqual(
            self.result_payloads()[-1]["error_message"],
            "Equipment is STOPPED",
        )

        self.adapter.process_control_command(
            {"command_id": 103, "action": "START"}
        )
        self.assertTrue(self.adapter.control_enabled)

    def test_stop_requests_active_navigation_cancel(self):
        class GoalHandle:
            def __init__(self):
                self.cancel_called = False

            def cancel_goal_async(self):
                self.cancel_called = True

                class Future:
                    def add_done_callback(self, callback):
                        pass

                return Future()

        goal_handle = GoalHandle()
        self.adapter.navigation_in_progress = True
        self.adapter.current_goal_handle = goal_handle

        self.adapter.process_control_command(
            {"command_id": 104, "action": "STOP"}
        )
        self.assertTrue(goal_handle.cancel_called)

    def test_lift_rejection_fails_immediately_with_mission_reason(self):
        self.adapter.process_lift_command({"command_id": 6, "action": "UP"})
        command = self.adapter.lift_command_publisher.messages[-1]
        self.assertEqual(json.loads(command.data), {"command_id": 6, "action": "UP"})
        message = FakeString()
        message.data = json.dumps({
            "command_id": 6, "action": "UP", "status": "REJECTED",
            "error_message": "Manual lift UP rejected: mission_state=LIFTING",
        })
        self.adapter.lift_result_callback(message)
        self.assertFalse(self.adapter.lift_in_progress)
        self.assertEqual(self.result_payloads()[-1]["status"], "FAILED")
        self.assertIn("mission_state=LIFTING", self.result_payloads()[-1]["error_message"])
        self.adapter.check_lift_timeout()
        self.assertEqual(len(self.result_payloads()), 2)

    def test_stale_lift_rejection_does_not_fail_new_command(self):
        self.adapter.process_lift_command({"command_id": 7, "action": "UP"})
        message = FakeString()
        message.data = json.dumps({"command_id": 6, "action": "UP", "status": "REJECTED"})
        self.adapter.lift_result_callback(message)
        self.assertTrue(self.adapter.lift_in_progress)
        message.data = "UP"
        self.adapter.lift_state_callback(message)
        self.assertEqual(self.result_payloads()[-1]["status"], "SUCCESS")

    def test_stop_blocks_manual_lift(self):
        self.adapter.process_control_command({"command_id": 8, "action": "STOP"})
        self.adapter.process_lift_command({"command_id": 9, "action": "UP"})
        self.assertFalse(self.adapter.lift_command_publisher.messages)
        self.assertEqual(self.result_payloads()[-1]["error_message"], "Equipment is STOPPED")


if __name__ == "__main__":
    unittest.main()
