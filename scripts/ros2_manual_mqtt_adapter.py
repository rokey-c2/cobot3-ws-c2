#!/usr/bin/env python3

import json
import os
import queue
import time

import paho.mqtt.client as mqtt
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node


MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
EQUIPMENT_CODE = os.getenv("EQUIPMENT_CODE", "AMR_IN")

ROS_CMD_VEL_TOPIC = os.getenv("ROS_CMD_VEL_TOPIC", "/cmd_vel")

MQTT_MANUAL_COMMAND_TOPIC = (
    f"controltower/command/amr/{EQUIPMENT_CODE}/manual"
)
MQTT_CONTROL_COMMAND_TOPIC = (
    f"controltower/command/equipment/{EQUIPMENT_CODE}/control"
)
MQTT_NAV_COMMAND_TOPIC = (
    f"controltower/command/amr/{EQUIPMENT_CODE}/navigate"
)

MANUAL_LINEAR_SPEED = float(os.getenv("MANUAL_LINEAR_SPEED", "0.90"))
MANUAL_BACKWARD_SPEED = float(os.getenv("MANUAL_BACKWARD_SPEED", "0.66"))
MANUAL_ANGULAR_SPEED = float(os.getenv("MANUAL_ANGULAR_SPEED", "0.50"))
MANUAL_DEADMAN_TIMEOUT = float(os.getenv("MANUAL_DEADMAN_TIMEOUT", "0.45"))
PUBLISH_PERIOD = 0.05

VALID_DIRECTIONS = {
    "FORWARD",
    "BACKWARD",
    "LEFT",
    "RIGHT",
    "STOP",
}


class ManualMqttAdapter(Node):

    def __init__(self):
        super().__init__("control_tower_manual_mqtt_adapter")

        self.velocity_publisher = self.create_publisher(
            Twist,
            ROS_CMD_VEL_TOPIC,
            10,
        )

        self.control_enabled = False
        self.manual_direction = "STOP"
        self.manual_deadline = 0.0
        self.message_queue = queue.Queue()

        self.mqtt_client = mqtt.Client(
            client_id="control-tower-manual-adapter",
        )
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.get_logger().info(
            f"Connecting MQTT broker: {MQTT_HOST}:{MQTT_PORT}"
        )
        self.mqtt_client.connect(
            MQTT_HOST,
            MQTT_PORT,
            keepalive=60,
        )
        self.mqtt_client.loop_start()

        self.control_timer = self.create_timer(
            PUBLISH_PERIOD,
            self.control_loop,
        )

        self.get_logger().info(
            f"ROS2 manual velocity publish: {ROS_CMD_VEL_TOPIC}"
        )
        self.get_logger().info(
            f"MQTT manual subscribe: {MQTT_MANUAL_COMMAND_TOPIC}"
        )
        self.get_logger().info(
            f"Manual speed: forward={MANUAL_LINEAR_SPEED:.2f} m/s "
            f"backward={MANUAL_BACKWARD_SPEED:.2f} m/s "
            f"turn={MANUAL_ANGULAR_SPEED:.2f} rad/s"
        )
        self.get_logger().info(
            f"Deadman timeout: {MANUAL_DEADMAN_TIMEOUT:.2f}s"
        )

    def on_mqtt_connect(
        self,
        client,
        userdata,
        flags,
        rc,
    ):
        print(
            f"[MANUAL][MQTT] Connected rc={rc}",
            flush=True,
        )

        for topic in (
            MQTT_MANUAL_COMMAND_TOPIC,
            MQTT_CONTROL_COMMAND_TOPIC,
            MQTT_NAV_COMMAND_TOPIC,
        ):
            client.subscribe(topic, qos=1)
            print(
                f"[MANUAL][MQTT] Subscribed: {topic}",
                flush=True,
            )

    def on_mqtt_message(
        self,
        client,
        userdata,
        message,
    ):
        try:
            payload = json.loads(
                message.payload.decode("utf-8")
            )
        except json.JSONDecodeError:
            self.get_logger().warning(
                f"Invalid JSON on {message.topic}"
            )
            return

        self.message_queue.put((message.topic, payload))

    def control_loop(self):
        while not self.message_queue.empty():
            topic, payload = self.message_queue.get()
            self.process_message(topic, payload)

        if self.manual_direction == "STOP":
            return

        if not self.control_enabled:
            self.stop_manual("equipment STOP")
            return

        if time.monotonic() > self.manual_deadline:
            self.stop_manual("deadman timeout")
            return

        self.publish_direction(self.manual_direction)

    def process_message(self, topic, payload):
        if topic == MQTT_CONTROL_COMMAND_TOPIC:
            action = str(payload.get("action", "")).strip().upper()

            if action == "START":
                self.control_enabled = True
                self.get_logger().info(
                    "Manual control enabled by AMR START"
                )

            elif action == "STOP":
                self.control_enabled = False
                self.stop_manual("AMR STOP")

            return

        if topic == MQTT_NAV_COMMAND_TOPIC:
            # 자동주행이 시작될 때 남아 있는 수동 속도를 반드시 지운다.
            self.stop_manual("navigation command received")
            return

        if topic != MQTT_MANUAL_COMMAND_TOPIC:
            return

        direction = str(
            payload.get("direction", "")
        ).strip().upper()

        if direction not in VALID_DIRECTIONS:
            self.get_logger().warning(
                f"Invalid manual direction: {direction}"
            )
            self.stop_manual("invalid direction")
            return

        if direction == "STOP":
            self.stop_manual("operator release")
            return

        if not self.control_enabled:
            self.get_logger().warning(
                f"Manual command ignored while AMR is STOPPED: {direction}"
            )
            self.stop_manual("AMR is STOPPED")
            return

        direction_changed = direction != self.manual_direction
        self.manual_direction = direction
        self.manual_deadline = (
            time.monotonic() + MANUAL_DEADMAN_TIMEOUT
        )

        if direction_changed:
            self.get_logger().info(
                f"Manual drive: {direction}"
            )

        self.publish_direction(direction)

    def publish_direction(self, direction):
        message = Twist()

        if direction == "FORWARD":
            message.linear.x = MANUAL_LINEAR_SPEED

        elif direction == "BACKWARD":
            message.linear.x = -MANUAL_BACKWARD_SPEED

        elif direction == "LEFT":
            message.angular.z = MANUAL_ANGULAR_SPEED

        elif direction == "RIGHT":
            message.angular.z = -MANUAL_ANGULAR_SPEED

        self.velocity_publisher.publish(message)

    def stop_manual(self, reason):
        was_active = self.manual_direction != "STOP"
        self.manual_direction = "STOP"
        self.manual_deadline = 0.0
        self.velocity_publisher.publish(Twist())

        if was_active:
            self.get_logger().info(
                f"Manual STOP: {reason}"
            )

    def destroy_node(self):
        self.stop_manual("adapter shutdown")
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        print(
            "[MANUAL][MQTT] Disconnected",
            flush=True,
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualMqttAdapter()

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
