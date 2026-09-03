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


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def info(self, message):
        pass

    def error(self, message):
        pass


class FakeNode:
    def __init__(self, name):
        pass

    def create_publisher(self, *args, **kwargs):
        return FakePublisher()

    def create_subscription(self, *args, **kwargs):
        return object()

    def create_timer(self, *args, **kwargs):
        return object()

    def get_logger(self):
        return FakeLogger()

    def destroy_node(self):
        pass


class FakeString:
    def __init__(self):
        self.data = ""


def load_module():
    mqtt_module = types.ModuleType("paho.mqtt.client")
    mqtt_module.Client = FakeMqttClient
    mqtt_module.CallbackAPIVersion = types.SimpleNamespace(VERSION2=2)
    paho_module = types.ModuleType("paho")
    paho_mqtt_module = types.ModuleType("paho.mqtt")
    paho_mqtt_module.client = mqtt_module
    paho_module.mqtt = paho_mqtt_module

    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_node_module.Node = FakeNode
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")
    std_msgs_msg_module.String = FakeString
    sys.modules.update(
        {
            "paho": paho_module,
            "paho.mqtt": paho_mqtt_module,
            "paho.mqtt.client": mqtt_module,
            "rclpy": rclpy_module,
            "rclpy.node": rclpy_node_module,
            "std_msgs": std_msgs_module,
            "std_msgs.msg": std_msgs_msg_module,
        }
    )

    path = Path(__file__).resolve().parents[1] / "scripts" / "process_mqtt_adapter.py"
    spec = importlib.util.spec_from_file_location("process_adapter_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ProcessMqttAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def setUp(self):
        self.adapter = self.module.Ros2ProcessMqttAdapter()

    def result_payloads(self):
        return [
            json.loads(payload)
            for topic, payload, qos, retain in self.adapter.mqtt_client.publications
            if topic == self.module.MQTT_COMMAND_RESULT_TOPIC
        ]

    def test_control_waits_for_real_isaac_feedback(self):
        self.adapter.process_control_command(
            "controltower/command/equipment/MAIN_CONVEYOR/control",
            {"command_id": 501, "action": "STOP"},
        )
        command = json.loads(
            self.adapter.equipment_command_publisher.messages[-1].data
        )
        self.assertEqual(
            command,
            {"equipment_code": "MAIN_CONVEYOR", "action": "STOP"},
        )
        self.assertEqual(self.result_payloads()[-1]["status"], "RUNNING")

        feedback = FakeString()
        feedback.data = json.dumps(
            {"equipment_code": "MAIN_CONVEYOR", "status": "STOPPED"}
        )
        self.adapter._on_ros_equipment_status(feedback)
        self.assertEqual(self.result_payloads()[-1]["status"], "SUCCESS")
        self.assertNotIn("MAIN_CONVEYOR", self.adapter.pending_commands)

    def test_repeated_p3020_state_is_emitted_once(self):
        status = FakeString()
        status.data = "SCANNING"
        self.adapter._on_p3020_state(status)
        self.adapter._on_p3020_state(status)
        events = [
            json.loads(payload)
            for topic, payload, qos, retain in self.adapter.mqtt_client.publications
            if topic == self.module.MQTT_PROCESS_EVENT_TOPIC
        ]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["equipment_code"], "P3020_IN")

    def test_p3020_arrival_command_waits_for_mission_ack(self):
        payload = {
            "command_id": 601, "mission_id": 9,
            "mission_code": "MISSION-9", "equipment_code": "AMR_IN",
        }
        self.adapter.process_p3020_arrival_command(payload)
        sent = json.loads(self.adapter.p3020_arrival_publisher.messages[-1].data)
        self.assertEqual(sent, payload)
        self.assertEqual(self.result_payloads()[-1]["status"], "RUNNING")

        state = FakeString()
        state.data = "REQUEST_CONVEYOR_DOCK"
        self.adapter._on_mission_state(state)
        self.assertEqual(self.result_payloads()[-1]["status"], "SUCCESS")
        self.assertIsNone(self.adapter.pending_arrival_command)

    def test_duplicate_p3020_arrival_is_busy(self):
        first = {"command_id": 601, "mission_id": 9, "mission_code": "M-9"}
        second = {"command_id": 602, "mission_id": 9, "mission_code": "M-9"}
        self.adapter.process_p3020_arrival_command(first)
        self.adapter.process_p3020_arrival_command(second)
        self.assertEqual(self.result_payloads()[-1]["status"], "BUSY")


if __name__ == "__main__":
    unittest.main()
