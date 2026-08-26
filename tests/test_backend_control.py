import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path


class FakeMessageInfo:
    rc = 0


class FakeMqttClient:
    def __init__(self, *args, **kwargs):
        self.connected = True
        self.publications = []

    def is_connected(self):
        return self.connected

    def publish(self, topic, payload, qos=0, retain=False):
        self.publications.append((topic, payload, qos, retain))
        return FakeMessageInfo()


class FakeCursor:
    def __init__(self):
        self.executions = []
        self.rowcount = 1
        self._fetchone = {
            "equipment_id": 1,
            "command_type": "STOP",
        }

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        pass

    def execute(self, query, params=None):
        self.executions.append((query, params))

    def fetchone(self):
        result = self._fetchone
        self._fetchone = None
        return result


class FakeConnection:
    def __init__(self):
        self.cursor_instance = FakeCursor()
        self.commits = 0

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        pass

    def cursor(self, **kwargs):
        return self.cursor_instance

    def commit(self):
        self.commits += 1


def load_mqtt_module():
    mqtt_module = types.ModuleType("paho.mqtt.client")
    mqtt_module.MQTT_ERR_SUCCESS = 0
    mqtt_module.CallbackAPIVersion = types.SimpleNamespace(VERSION2=2)
    mqtt_module.Client = FakeMqttClient

    paho_module = types.ModuleType("paho")
    paho_mqtt_module = types.ModuleType("paho.mqtt")
    paho_mqtt_module.client = mqtt_module
    paho_module.mqtt = paho_mqtt_module

    psycopg_module = types.ModuleType("psycopg")
    psycopg_rows_module = types.ModuleType("psycopg.rows")
    psycopg_rows_module.dict_row = object()

    database_module = types.ModuleType("app.database")
    database_module.get_db_connection = lambda: FakeConnection()
    app_module = types.ModuleType("app")

    sys.modules.update(
        {
            "paho": paho_module,
            "paho.mqtt": paho_mqtt_module,
            "paho.mqtt.client": mqtt_module,
            "psycopg": psycopg_module,
            "psycopg.rows": psycopg_rows_module,
            "app": app_module,
            "app.database": database_module,
        }
    )

    path = (
        Path(__file__).resolve().parents[1]
        / "backend"
        / "app"
        / "mqtt_client.py"
    )
    spec = importlib.util.spec_from_file_location(
        "backend_mqtt_under_test",
        path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, database_module


class BackendControlTest(unittest.TestCase):
    def setUp(self):
        self.module, self.database_module = load_mqtt_module()

    def test_control_publish_uses_equipment_topic_and_retains_last_state(self):
        self.module.publish_equipment_control_command(
            equipment_code="AMR_IN",
            command_id=55,
            action="stop",
        )

        topic, raw_payload, qos, retain = (
            self.module.mqtt_client.publications[-1]
        )
        self.assertEqual(
            topic,
            "controltower/command/equipment/AMR_IN/control",
        )
        self.assertEqual(
            json.loads(raw_payload),
            {"command_id": 55, "action": "STOP"},
        )
        self.assertEqual(qos, 1)
        self.assertTrue(retain)

    def test_successful_stop_result_updates_command_and_equipment_state(self):
        connection = FakeConnection()
        self.module.get_db_connection = lambda: connection

        self.module.update_command_result(
            {
                "command_id": 55,
                "equipment_code": "AMR_IN",
                "command_type": "STOP",
                "status": "SUCCESS",
            }
        )

        executions = connection.cursor_instance.executions
        self.assertEqual(len(executions), 2)
        self.assertIn("UPDATE equipment_command", executions[0][0])
        self.assertIn("UPDATE equipment_state", executions[1][0])
        self.assertEqual(executions[1][1], ("STOPPED", 1))
        self.assertEqual(connection.commits, 1)


if __name__ == "__main__":
    unittest.main()
