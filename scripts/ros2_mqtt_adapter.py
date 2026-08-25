#!/usr/bin/env python3

import json
import math
import os
import queue
import time

import paho.mqtt.client as mqtt
import rclpy

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

ROS_ODOM_TOPIC = "/chassis/odom"
ROS_NAV_ACTION = "/navigate_to_pose"

MQTT_ODOM_TOPIC = "controltower/amr/AMR_IN/odom"
MQTT_NAV_COMMAND_TOPIC = "controltower/command/amr/AMR_IN/navigate"
MQTT_COMMAND_RESULT_TOPIC = "controltower/result/command"

PUBLISH_INTERVAL = 0.5


class Ros2MqttAdapter(Node):

    def __init__(self):
        super().__init__("control_tower_ros2_mqtt_adapter")

        # -------------------------------------------------
        # MQTT
        # Host Ubuntu의 paho-mqtt 1.x와 호환되는 방식
        # -------------------------------------------------
        self.mqtt_client = mqtt.Client(
            client_id="control-tower-ros2-adapter",
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

        # -------------------------------------------------
        # ROS2 Odom
        # -------------------------------------------------
        self.odom_subscription = self.create_subscription(
            Odometry,
            ROS_ODOM_TOPIC,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.last_publish_time = 0.0

        # -------------------------------------------------
        # Nav2 Action Client
        # -------------------------------------------------
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            ROS_NAV_ACTION,
        )

        # MQTT callback은 별도 thread에서 실행되므로,
        # ROS Action 실행은 queue를 통해 ROS thread에서 처리한다.
        self.command_queue = queue.Queue()

        self.navigation_in_progress = False
        self.current_command_id = None

        self.command_timer = self.create_timer(
            0.1,
            self.process_command_queue,
        )

        self.get_logger().info(
            f"ROS2 subscribe: {ROS_ODOM_TOPIC}"
        )

        self.get_logger().info(
            f"MQTT publish: {MQTT_ODOM_TOPIC}"
        )

        self.get_logger().info(
            f"MQTT subscribe: {MQTT_NAV_COMMAND_TOPIC}"
        )

        self.get_logger().info(
            f"Nav2 action: {ROS_NAV_ACTION}"
        )

    # =====================================================
    # MQTT
    # =====================================================

    def on_mqtt_connect(
        self,
        client,
        userdata,
        flags,
        rc,
    ):
        print(
            f"[MQTT] Connected rc={rc}",
            flush=True,
        )

        client.subscribe(
            MQTT_NAV_COMMAND_TOPIC
        )

        print(
            f"[MQTT] Subscribed: {MQTT_NAV_COMMAND_TOPIC}",
            flush=True,
        )

    def on_mqtt_message(
        self,
        client,
        userdata,
        message,
    ):
        raw_payload = message.payload.decode("utf-8")

        print(
            f"[MQTT] command topic={message.topic} "
            f"payload={raw_payload}",
            flush=True,
        )

        if message.topic != MQTT_NAV_COMMAND_TOPIC:
            return

        try:
            payload = json.loads(raw_payload)

        except json.JSONDecodeError:
            print(
                "[MQTT] Invalid NAVIGATE JSON",
                flush=True,
            )
            return

        self.command_queue.put(payload)

    # =====================================================
    # ODOM
    # =====================================================

    def quaternion_to_yaw(
        self,
        x,
        y,
        z,
        w,
    ):
        siny_cosp = 2.0 * (
            w * z
            + x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y
            + z * z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    def odom_callback(
        self,
        message: Odometry,
    ):
        current_time = time.monotonic()

        if (
            current_time - self.last_publish_time
            < PUBLISH_INTERVAL
        ):
            return

        self.last_publish_time = current_time

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        payload = {
            "x": float(position.x),
            "y": float(position.y),
            "yaw": float(yaw),
        }

        self.mqtt_client.publish(
            MQTT_ODOM_TOPIC,
            json.dumps(payload),
        )

        self.get_logger().info(
            "Published AMR_IN odom "
            f"x={payload['x']:.3f} "
            f"y={payload['y']:.3f} "
            f"yaw={payload['yaw']:.3f}"
        )

    # =====================================================
    # NAVIGATION COMMAND
    # =====================================================

    def process_command_queue(self):

        if self.command_queue.empty():
            return

        payload = self.command_queue.get()

        command_id = payload.get("command_id")
        x = payload.get("x")
        y = payload.get("y")
        yaw = payload.get("yaw", 0.0)

        if (
            command_id is None
            or x is None
            or y is None
        ):
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                error_message="Invalid NAVIGATE payload",
            )
            return

        if self.navigation_in_progress:
            self.publish_command_result(
                command_id=command_id,
                status="BUSY",
                error_message="Navigation already in progress",
            )
            return

        self.send_navigation_goal(
            command_id=command_id,
            x=float(x),
            y=float(y),
            yaw=float(yaw),
        )

    def send_navigation_goal(
        self,
        command_id,
        x,
        y,
        yaw,
    ):
        self.get_logger().info(
            f"NAVIGATE command_id={command_id} "
            f"target=({x:.3f}, {y:.3f}, {yaw:.3f})"
        )

        if not self.navigate_client.wait_for_server(
            timeout_sec=3.0
        ):
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                error_message="Nav2 action server unavailable",
            )
            return

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # yaw → quaternion
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )
        goal.pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        self.navigation_in_progress = True
        self.current_command_id = command_id

        future = self.navigate_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.navigation_goal_response_callback
        )

    def navigation_goal_response_callback(
        self,
        future,
    ):
        goal_handle = future.result()

        command_id = self.current_command_id

        if not goal_handle.accepted:
            self.get_logger().error(
                f"Nav2 goal rejected: command_id={command_id}"
            )

            self.navigation_in_progress = False
            self.current_command_id = None

            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                error_message="Nav2 goal rejected",
            )
            return

        self.get_logger().info(
            f"Nav2 goal accepted: command_id={command_id}"
        )

        self.publish_command_result(
            command_id=command_id,
            status="RUNNING",
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    def navigation_result_callback(
        self,
        future,
    ):
        wrapped_result = future.result()

        command_id = self.current_command_id
        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            final_status = "SUCCESS"
            error_message = None

            self.get_logger().info(
                f"Nav2 SUCCEEDED: command_id={command_id}"
            )

        else:
            final_status = "FAILED"
            error_message = (
                f"Nav2 finished with status={status}"
            )

            self.get_logger().error(
                f"Nav2 FAILED: command_id={command_id} "
                f"status={status}"
            )

        self.navigation_in_progress = False
        self.current_command_id = None

        self.publish_command_result(
            command_id=command_id,
            status=final_status,
            error_message=error_message,
        )

    # =====================================================
    # COMMAND RESULT
    # =====================================================

    def publish_command_result(
        self,
        command_id,
        status,
        error_message=None,
    ):
        payload = {
            "command_id": command_id,
            "equipment_code": "AMR_IN",
            "command_type": "NAVIGATE",
            "status": status,
        }

        if error_message is not None:
            payload["error_message"] = error_message

        self.mqtt_client.publish(
            MQTT_COMMAND_RESULT_TOPIC,
            json.dumps(payload),
        )

        print(
            f"[MQTT] result={payload}",
            flush=True,
        )

    # =====================================================
    # SHUTDOWN
    # =====================================================

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

        print(
            "[MQTT] Disconnected",
            flush=True,
        )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = Ros2MqttAdapter()

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
