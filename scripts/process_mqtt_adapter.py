#!/usr/bin/env python3
"""Bridge live Isaac process states to MQTT without requiring Nav2.

Run this beside ``ros2_mqtt_adapter.py``.  The existing adapter remains the
owner of AMR navigation/lift/control; this process owns P3020, conveyor, and
sorter START/STOP plus the mission/package event stream.
"""

import json
import os
import queue
import time
import uuid

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_COMMAND_TOPIC = "controltower/command/equipment/+/control"
MQTT_COMMAND_RESULT_TOPIC = "controltower/result/command"
MQTT_PROCESS_EVENT_TOPIC = "controltower/process/event"
MQTT_P3020_ARRIVAL_TOPIC = "controltower/command/mission/p3020-arrival"

ROS_EQUIPMENT_COMMAND_TOPIC = "/controltower/equipment/command"
ROS_EQUIPMENT_STATUS_TOPIC = "/controltower/equipment/status"
ROS_PROCESS_EVENT_TOPIC = "/controltower/process/event"
ROS_P3020_ARRIVAL_TOPIC = "/amr_a/p3020_arrival_confirm"

SUPPORTED_EQUIPMENT = {
    "P3020_IN",
    "MAIN_CONVEYOR",
    "SORTER_A",
    "SORTER_B",
    "SORTER_C",
}
COMMAND_TIMEOUT_SECONDS = float(os.getenv("PROCESS_COMMAND_TIMEOUT", "8.0"))


class Ros2ProcessMqttAdapter(Node):
    def __init__(self):
        super().__init__("control_tower_process_mqtt_adapter")
        self.message_queue = queue.Queue()
        self.pending_commands = {}
        self.last_process_state = {}
        self.pending_arrival_command = None

        self.equipment_command_publisher = self.create_publisher(
            String, ROS_EQUIPMENT_COMMAND_TOPIC, 10
        )
        self.p3020_arrival_publisher = self.create_publisher(
            String, ROS_P3020_ARRIVAL_TOPIC, 10
        )
        self.create_subscription(
            String, ROS_EQUIPMENT_STATUS_TOPIC,
            self._on_ros_equipment_status, 10,
        )
        self.create_subscription(
            String, ROS_PROCESS_EVENT_TOPIC,
            self._on_ros_process_event, 10,
        )
        self.create_subscription(
            String, "/amr_a/pickup_state", self._on_amr_state, 10
        )
        self.create_subscription(
            String, "/amr_a/mission_state", self._on_mission_state, 10
        )
        self.create_subscription(
            String, "/arm_a/pick_place_status", self._on_p3020_state, 10
        )
        self.create_subscription(
            String, "/arm_b/pick_place_status", self._on_out_state, 10
        )

        self.mqtt_client = mqtt.Client(
            client_id=f"process-ros2-adapter-{os.getpid()}",
        )
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
        self.mqtt_client.loop_start()

        self.create_timer(0.05, self._drain_mqtt_queue)
        self.create_timer(0.5, self._expire_commands)
        self.get_logger().info(
            "Process bridge ready (Nav2 not required): "
            f"{ROS_EQUIPMENT_COMMAND_TOPIC} -> Isaac, process states -> MQTT"
        )

    def _on_mqtt_connect(self, client, userdata, flags, reason_code):
        del userdata, flags
        self.get_logger().info(
            f"MQTT connected {MQTT_HOST}:{MQTT_PORT}, reason={reason_code}"
        )
        client.subscribe(MQTT_COMMAND_TOPIC, qos=1)
        client.subscribe(MQTT_P3020_ARRIVAL_TOPIC, qos=1)

    def _on_mqtt_message(self, client, userdata, message):
        del client, userdata
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            self.get_logger().error(f"Invalid MQTT payload: {error}")
            return
        self.message_queue.put((message.topic, payload))

    def _drain_mqtt_queue(self):
        while True:
            try:
                topic, payload = self.message_queue.get_nowait()
            except queue.Empty:
                return
            if topic == MQTT_P3020_ARRIVAL_TOPIC:
                self.process_p3020_arrival_command(payload)
            else:
                self.process_control_command(topic, payload)

    def process_p3020_arrival_command(self, payload):
        command_id = payload.get("command_id")
        mission_id = payload.get("mission_id")
        mission_code = str(payload.get("mission_code", "")).strip()
        equipment_code = str(payload.get("equipment_code", "AMR_IN")).strip().upper()
        if command_id is None or mission_id is None or not mission_code:
            self.publish_command_result(
                command_id, equipment_code, "CONFIRM_P3020_ARRIVAL", "FAILED",
                "Invalid P3020 arrival payload",
            )
            return
        if self.pending_arrival_command is not None:
            self.publish_command_result(
                command_id, equipment_code, "CONFIRM_P3020_ARRIVAL", "BUSY",
                "Another P3020 arrival confirmation is in progress",
            )
            return

        message = String()
        message.data = json.dumps(payload)
        self.p3020_arrival_publisher.publish(message)
        self.pending_arrival_command = {
            "command_id": int(command_id),
            "equipment_code": equipment_code,
            "deadline": time.monotonic() + COMMAND_TIMEOUT_SECONDS,
        }
        self.publish_command_result(
            command_id, equipment_code, "CONFIRM_P3020_ARRIVAL", "RUNNING", None,
        )

    def _on_mission_state(self, message):
        pending = self.pending_arrival_command
        if pending is None:
            return
        state = str(message.data).strip().upper()
        accepted_states = {
            "REQUEST_CONVEYOR_DOCK", "REQUEST_LOWER_AT_DELIVERY",
            "P3020_START", "P3020_GOAL_SENT", "P3020_WORKING",
        }
        if state in accepted_states:
            self.publish_command_result(
                pending["command_id"], pending["equipment_code"],
                "CONFIRM_P3020_ARRIVAL", "SUCCESS", None,
            )
            self.pending_arrival_command = None
        elif state == "ERROR":
            self.publish_command_result(
                pending["command_id"], pending["equipment_code"],
                "CONFIRM_P3020_ARRIVAL", "FAILED", "Mission entered ERROR",
            )
            self.pending_arrival_command = None

    def process_control_command(self, topic, payload):
        parts = topic.split("/")
        if len(parts) != 5:
            return
        equipment_code = parts[3].strip().upper()
        if equipment_code not in SUPPORTED_EQUIPMENT:
            return

        command_id = payload.get("command_id")
        action = str(payload.get("action", "")).strip().upper()
        if command_id is None or action not in {"START", "STOP"}:
            self.publish_command_result(
                command_id, equipment_code, action or "CONTROL", "FAILED",
                "Invalid equipment control payload",
            )
            return

        message = String()
        message.data = json.dumps(
            {"equipment_code": equipment_code, "action": action}
        )
        self.equipment_command_publisher.publish(message)
        self.pending_commands[equipment_code] = {
            "command_id": int(command_id),
            "action": action,
            "target": "RUNNING" if action == "START" else "STOPPED",
            "deadline": time.monotonic() + COMMAND_TIMEOUT_SECONDS,
        }
        self.publish_command_result(
            command_id, equipment_code, action, "RUNNING", None
        )

    def _on_ros_equipment_status(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"Invalid Isaac equipment status: {error}")
            return

        equipment_code = str(payload.get("equipment_code", "")).strip().upper()
        status = str(payload.get("status", "")).strip().upper()
        if equipment_code not in SUPPORTED_EQUIPMENT or not status:
            return

        self.mqtt_client.publish(
            f"controltower/equipment/{equipment_code}/status",
            json.dumps(payload), qos=1,
        )
        pending = self.pending_commands.get(equipment_code)
        if pending and status == pending["target"]:
            self.publish_command_result(
                pending["command_id"], equipment_code,
                pending["action"], "SUCCESS", None,
            )
            del self.pending_commands[equipment_code]

    def _on_ros_process_event(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"Invalid Isaac process event: {error}")
            return
        self.publish_process_event(payload)

    def _publish_state_change(self, source, state, payload):
        if self.last_process_state.get(source) == state:
            return
        self.last_process_state[source] = state
        payload = dict(payload)
        payload.update(
            {
                "state": state,
                "event_key": f"{source}:{uuid.uuid4().hex}",
                "observed_at": time.time(),
            }
        )
        self.publish_process_event(payload)

    def _on_amr_state(self, message):
        state = message.data.strip().upper()
        tracked_states = {
            "ROTATE_TO_DOCK",
            "ENTER_CARGO",
            "LIFTING",
            "PICKUP_DONE",
            "CONVEYOR_DOCK_DONE",
            "ERROR",
        }
        if state not in tracked_states:
            return
        self._publish_state_change(
            "amr", state,
            {"event_type": "AMR_STATE", "equipment_code": "AMR_IN"},
        )

    def _on_p3020_state(self, message):
        state = message.data.strip()
        if not state:
            return
        self._publish_state_change(
            "p3020", state,
            {"event_type": "P3020_STATE", "equipment_code": "P3020_IN"},
        )

    def _on_out_state(self, message):
        state = message.data.strip()
        if state not in {"BIN_PLACED", "DONE_SUCCESS", "DONE_FAIL:RETREAT_FAILED", "DONE_FAIL:HOME_RETURN_FAILED"}:
            return
        self._publish_state_change(
            "p3020_out", state,
            {"event_type": "P3020_OUT_STATE", "equipment_code": "P3020_OUT", "region": "D"},
        )

    def publish_process_event(self, payload):
        payload = dict(payload)
        payload.setdefault("event_key", f"process:{uuid.uuid4().hex}")
        payload.setdefault("observed_at", time.time())
        self.mqtt_client.publish(
            MQTT_PROCESS_EVENT_TOPIC,
            json.dumps(payload, ensure_ascii=False),
            qos=1,
        )

    def _expire_commands(self):
        now = time.monotonic()
        if (self.pending_arrival_command is not None and
                self.pending_arrival_command["deadline"] <= now):
            pending = self.pending_arrival_command
            self.pending_arrival_command = None
            self.publish_command_result(
                pending["command_id"], pending["equipment_code"],
                "CONFIRM_P3020_ARRIVAL", "FAILED",
                "Timed out waiting for ROS2 mission acknowledgement",
            )
        expired = [
            code for code, command in self.pending_commands.items()
            if command["deadline"] <= now
        ]
        for equipment_code in expired:
            pending = self.pending_commands.pop(equipment_code)
            self.publish_command_result(
                pending["command_id"], equipment_code,
                pending["action"], "FAILED",
                "Timed out waiting for Isaac equipment feedback",
            )

    def publish_command_result(
        self, command_id, equipment_code, command_type, status, error_message
    ):
        if command_id is None:
            return
        payload = {
            "command_id": int(command_id),
            "equipment_code": equipment_code,
            "command_type": command_type,
            "status": status,
            "error_message": error_message,
        }
        self.mqtt_client.publish(
            MQTT_COMMAND_RESULT_TOPIC, json.dumps(payload), qos=1
        )

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ros2ProcessMqttAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
