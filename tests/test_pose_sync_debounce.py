import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path


class DummyNode:
    pass


class DummyMessage:
    pass


class DummyQoSProfile:
    def __init__(self, depth=1):
        self.depth = depth


class DummyTime:
    pass


def load_module():
    mqtt_module = types.ModuleType("paho.mqtt.client")
    paho_module = types.ModuleType("paho")
    paho_mqtt_module = types.ModuleType("paho.mqtt")
    paho_mqtt_module.client = mqtt_module
    paho_module.mqtt = paho_mqtt_module

    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_node_module.Node = DummyNode
    rclpy_qos_module = types.ModuleType("rclpy.qos")
    rclpy_qos_module.DurabilityPolicy = types.SimpleNamespace(
        TRANSIENT_LOCAL=1
    )
    rclpy_qos_module.QoSProfile = DummyQoSProfile
    rclpy_qos_module.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    rclpy_time_module = types.ModuleType("rclpy.time")
    rclpy_time_module.Time = DummyTime

    geometry_module = types.ModuleType("geometry_msgs")
    geometry_msg_module = types.ModuleType("geometry_msgs.msg")
    geometry_msg_module.PoseStamped = DummyMessage
    geometry_msg_module.PoseWithCovarianceStamped = DummyMessage

    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")
    std_msgs_msg_module.String = DummyMessage

    tf2_module = types.ModuleType("tf2_ros")
    tf2_module.Buffer = object
    tf2_module.TransformListener = object

    sys.modules.update(
        {
            "paho": paho_module,
            "paho.mqtt": paho_mqtt_module,
            "paho.mqtt.client": mqtt_module,
            "rclpy": rclpy_module,
            "rclpy.node": rclpy_node_module,
            "rclpy.qos": rclpy_qos_module,
            "rclpy.time": rclpy_time_module,
            "geometry_msgs": geometry_module,
            "geometry_msgs.msg": geometry_msg_module,
            "std_msgs": std_msgs_module,
            "std_msgs.msg": std_msgs_msg_module,
            "tf2_ros": tf2_module,
        }
    )

    path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "pose_sync_manager.py"
    )
    spec = importlib.util.spec_from_file_location(
        "pose_sync_manager_under_test",
        path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeTfBuffer:
    def __init__(self, x, y, yaw=0.0):
        self.set_pose(x, y, yaw)

    def set_pose(self, x, y, yaw=0.0):
        self.transform = types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=x, y=y),
                rotation=types.SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            )
        )

    def lookup_transform(self, *args):
        return self.transform


class PoseSyncDebounceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()
        cls.module.POSE_TIMEOUT = 100.0
        cls.module.DESYNC_DEBOUNCE_SECONDS = 2.0

    def make_manager(self):
        manager = object.__new__(self.module.PoseSyncManager)
        manager.latest_pose = (1.0, 2.0, 0.0)
        manager.last_pose_rx_time = 10.0
        manager.restore_state = self.module.RESTORE_READY
        manager.nav2_initial_pose_subscriber_available = True
        manager.nav2_needs_initial_pose = False
        manager.initial_pose_retries_remaining = 0
        manager.last_sync_status = "SYNCED"
        manager.desync_started_at = None
        manager.tf_buffer = FakeTfBuffer(0.7, 2.0)
        manager.published = []

        def publish(status, position_error=None, yaw_error=None):
            manager.last_sync_status = status
            manager.published.append(status)

        manager.publish_sync_status = publish
        return manager

    def test_short_mismatch_keeps_synced(self):
        manager = self.make_manager()

        self.module.time.monotonic = lambda: 10.0
        manager.check_sync()
        self.module.time.monotonic = lambda: 11.9
        manager.check_sync()

        self.assertEqual(manager.last_sync_status, "SYNCED")
        self.assertEqual(manager.published, [])

    def test_continuous_mismatch_becomes_desync_after_two_seconds(self):
        manager = self.make_manager()

        self.module.time.monotonic = lambda: 10.0
        manager.check_sync()
        self.module.time.monotonic = lambda: 12.0
        manager.check_sync()

        self.assertEqual(manager.last_sync_status, "DESYNC")
        self.assertEqual(manager.published, ["DESYNC"])

    def test_matching_pose_resets_debounce_immediately(self):
        manager = self.make_manager()

        self.module.time.monotonic = lambda: 10.0
        manager.check_sync()
        manager.tf_buffer.set_pose(1.0, 2.0)
        self.module.time.monotonic = lambda: 11.0
        manager.check_sync()

        self.assertIsNone(manager.desync_started_at)
        self.assertEqual(manager.last_sync_status, "SYNCED")
        self.assertEqual(manager.published, ["SYNCED"])

        manager.tf_buffer.set_pose(0.7, 2.0)
        self.module.time.monotonic = lambda: 12.0
        manager.check_sync()
        self.module.time.monotonic = lambda: 13.9
        manager.check_sync()

        self.assertEqual(manager.last_sync_status, "SYNCED")
        self.assertNotIn("DESYNC", manager.published)


if __name__ == "__main__":
    unittest.main()
