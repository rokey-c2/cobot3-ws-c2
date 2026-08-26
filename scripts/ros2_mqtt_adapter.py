#!/usr/bin/env python3

import json
import math
import os
import queue
import time

import paho.mqtt.client as mqtt
import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

EQUIPMENT_CODE = os.getenv("EQUIPMENT_CODE", "AMR_IN")

ROS_ODOM_TOPIC = os.getenv("ROS_ODOM_TOPIC", "/chassis/odom")
ROS_NAV_ACTION = os.getenv("ROS_NAV_ACTION", "/navigate_to_pose")
ROS_CMD_VEL_TOPIC = os.getenv("ROS_CMD_VEL_TOPIC", "/cmd_vel")
ROS_LIFT_COMMAND_TOPIC = os.getenv(
    "ROS_LIFT_COMMAND_TOPIC",
    "/amr_a/lift_command",
)
ROS_LIFT_STATE_TOPIC = os.getenv(
    "ROS_LIFT_STATE_TOPIC",
    "/amr_a/lift_state",
)

MQTT_ODOM_TOPIC = f"controltower/amr/{EQUIPMENT_CODE}/odom"
MQTT_STATUS_TOPIC = f"controltower/amr/{EQUIPMENT_CODE}/status"
MQTT_NAV_COMMAND_TOPIC = (
    f"controltower/command/amr/{EQUIPMENT_CODE}/navigate"
)
MQTT_LIFT_COMMAND_TOPIC = (
    f"controltower/command/amr/{EQUIPMENT_CODE}/lift"
)
MQTT_CONTROL_COMMAND_TOPIC = (
    f"controltower/command/equipment/{EQUIPMENT_CODE}/control"
)
MQTT_LIFT_STATE_TOPIC = f"controltower/amr/{EQUIPMENT_CODE}/lift"
MQTT_COMMAND_RESULT_TOPIC = "controltower/result/command"

PUBLISH_INTERVAL = 0.5
LIFT_COMMAND_TIMEOUT = 10.0


class Ros2MqttAdapter(Node):

    def __init__(self):
        super().__init__("control_tower_ros2_mqtt_adapter")

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

        self.odom_subscription = self.create_subscription(
            Odometry,
            ROS_ODOM_TOPIC,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.last_publish_time = 0.0

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            ROS_NAV_ACTION,
        )
        self.navigation_in_progress = False
        self.current_command_id = None
        self.current_goal_handle = None
        self.cancel_navigation_when_accepted = False

        self.stop_velocity_publisher = self.create_publisher(
            Twist,
            ROS_CMD_VEL_TOPIC,
            10,
        )

        self.lift_command_publisher = self.create_publisher(
            String,
            ROS_LIFT_COMMAND_TOPIC,
            10,
        )
        self.lift_state_subscription = self.create_subscription(
            String,
            ROS_LIFT_STATE_TOPIC,
            self.lift_state_callback,
            10,
        )
        self.current_lift_state = None
        self.lift_in_progress = False
        self.current_lift_command_id = None
        self.current_lift_action = None
        self.lift_command_deadline = None

        # 기존 동작 호환을 위해 처음에는 제어 허용으로 시작한다. MQTT의
        # retain된 마지막 STOP 명령이 있으면 연결 직후 다시 STOP으로 복구된다.
        self.control_enabled = True
        self.control_command_queue = queue.Queue()
        self.command_queue = queue.Queue()

        self.command_timer = self.create_timer(
            0.1,
            self.process_command_queue,
        )
        self.stop_timer = self.create_timer(
            0.05,
            self.enforce_control_stop,
        )

        self.get_logger().info(f"ROS2 subscribe: {ROS_ODOM_TOPIC}")
        self.get_logger().info(f"MQTT publish: {MQTT_ODOM_TOPIC}")
        self.get_logger().info(
            f"MQTT subscribe: {MQTT_NAV_COMMAND_TOPIC}"
        )
        self.get_logger().info(f"Nav2 action: {ROS_NAV_ACTION}")
        self.get_logger().info(
            f"ROS2 STOP velocity publish: {ROS_CMD_VEL_TOPIC}"
        )
        self.get_logger().info(
            f"ROS2 publish: {ROS_LIFT_COMMAND_TOPIC}"
        )
        self.get_logger().info(
            f"ROS2 subscribe: {ROS_LIFT_STATE_TOPIC}"
        )
        self.get_logger().info(
            f"MQTT subscribe: {MQTT_LIFT_COMMAND_TOPIC}"
        )
        self.get_logger().info(
            f"MQTT subscribe: {MQTT_CONTROL_COMMAND_TOPIC}"
        )

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

        for topic in (
            MQTT_NAV_COMMAND_TOPIC,
            MQTT_LIFT_COMMAND_TOPIC,
            MQTT_CONTROL_COMMAND_TOPIC,
        ):
            client.subscribe(topic, qos=1)
            print(
                f"[MQTT] Subscribed: {topic}",
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

        if message.topic not in {
            MQTT_NAV_COMMAND_TOPIC,
            MQTT_LIFT_COMMAND_TOPIC,
            MQTT_CONTROL_COMMAND_TOPIC,
        }:
            return

        try:
            payload = json.loads(raw_payload)
        except json.JSONDecodeError:
            print(
                f"[MQTT] Invalid JSON topic={message.topic}",
                flush=True,
            )
            return

        if message.topic == MQTT_CONTROL_COMMAND_TOPIC:
            # STOP은 일반 동작 명령보다 먼저 처리한다.
            self.control_command_queue.put(payload)
            return

        if message.topic == MQTT_NAV_COMMAND_TOPIC:
            self.command_queue.put(("NAVIGATE", payload))
            return

        if message.topic == MQTT_LIFT_COMMAND_TOPIC:
            self.command_queue.put(("LIFT", payload))

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, message: Odometry):
        current_time = time.monotonic()

        if current_time - self.last_publish_time < PUBLISH_INTERVAL:
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
            f"Published {EQUIPMENT_CODE} odom "
            f"x={payload['x']:.3f} "
            f"y={payload['y']:.3f} "
            f"yaw={payload['yaw']:.3f}"
        )

    def process_command_queue(self):
        self.check_lift_timeout()

        if not self.control_command_queue.empty():
            self.process_control_command(
                self.control_command_queue.get()
            )
            return

        if self.command_queue.empty():
            return

        command_type, payload = self.command_queue.get()

        if command_type == "NAVIGATE":
            self.process_navigation_command(payload)
            return

        if command_type == "LIFT":
            self.process_lift_command(payload)

    def process_control_command(self, payload):
        command_id = payload.get("command_id")
        action = str(payload.get("action", "")).strip().upper()

        if command_id is None or action not in {"START", "STOP"}:
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                command_type="CONTROL",
                error_message="Invalid START/STOP payload",
            )
            return

        self.publish_command_result(
            command_id=command_id,
            status="RUNNING",
            command_type=action,
        )

        if action == "STOP":
            self.control_enabled = False
            self.cancel_active_navigation()
            self.publish_zero_velocity()
            self.publish_equipment_status("STOPPED")

            self.get_logger().warning(
                f"STOP applied: command_id={command_id}; "
                "new Navigate/Lift commands are blocked"
            )

        else:
            self.control_enabled = True
            if not self.navigation_in_progress:
                self.cancel_navigation_when_accepted = False
            self.publish_equipment_status("RUNNING")

            self.get_logger().info(
                f"START applied: command_id={command_id}; "
                "new commands are enabled"
            )

        self.publish_command_result(
            command_id=command_id,
            status="SUCCESS",
            command_type=action,
        )

    def publish_equipment_status(self, status):
        payload = {
            "status": status,
            "mode": "AUTO",
        }
        self.mqtt_client.publish(
            MQTT_STATUS_TOPIC,
            json.dumps(payload),
            qos=1,
            retain=True,
        )

    def publish_zero_velocity(self):
        self.stop_velocity_publisher.publish(Twist())

    def enforce_control_stop(self):
        if not self.control_enabled:
            self.publish_zero_velocity()

    def cancel_active_navigation(self):
        if not self.navigation_in_progress:
            return

        if self.current_goal_handle is None:
            self.cancel_navigation_when_accepted = True
            self.get_logger().warning(
                "STOP received while Nav2 goal acceptance is pending"
            )
            return

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            self.navigation_cancel_response_callback
        )

    def navigation_cancel_response_callback(self, future):
        try:
            response = future.result()
            cancel_count = len(response.goals_canceling)
            self.get_logger().warning(
                f"Nav2 cancel response: goals_canceling={cancel_count}"
            )
        except Exception as error:
            self.get_logger().error(
                f"Nav2 cancel request failed: {error}"
            )

    def process_navigation_command(self, payload):
        command_id = payload.get("command_id")
        x = payload.get("x")
        y = payload.get("y")
        yaw = payload.get("yaw", 0.0)

        if command_id is None or x is None or y is None:
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                error_message="Invalid NAVIGATE payload",
            )
            return

        if not self.control_enabled:
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                error_message="Equipment is STOPPED",
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

    def send_navigation_goal(self, command_id, x, y, yaw):
        self.get_logger().info(
            f"NAVIGATE command_id={command_id} "
            f"target=({x:.3f}, {y:.3f}, {yaw:.3f})"
        )

        if not self.navigate_client.wait_for_server(timeout_sec=3.0):
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                error_message="Nav2 action server unavailable",
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.navigation_in_progress = True
        self.current_command_id = command_id
        self.current_goal_handle = None

        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(
            self.navigation_goal_response_callback
        )

    def navigation_goal_response_callback(self, future):
        goal_handle = future.result()
        command_id = self.current_command_id

        if not goal_handle.accepted:
            self.get_logger().error(
                f"Nav2 goal rejected: command_id={command_id}"
            )
            self.clear_navigation_command()
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                error_message="Nav2 goal rejected",
            )
            return

        self.current_goal_handle = goal_handle
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

        if (
            not self.control_enabled
            or self.cancel_navigation_when_accepted
        ):
            self.cancel_navigation_when_accepted = False
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                self.navigation_cancel_response_callback
            )

    def navigation_result_callback(self, future):
        wrapped_result = future.result()
        command_id = self.current_command_id
        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            final_status = "SUCCESS"
            error_message = None
            self.get_logger().info(
                f"Nav2 SUCCEEDED: command_id={command_id}"
            )

        elif status == GoalStatus.STATUS_CANCELED:
            final_status = "FAILED"
            error_message = "Navigation canceled by STOP"
            self.get_logger().warning(
                f"Nav2 CANCELED: command_id={command_id}"
            )

        else:
            final_status = "FAILED"
            error_message = f"Nav2 finished with status={status}"
            self.get_logger().error(
                f"Nav2 FAILED: command_id={command_id} status={status}"
            )

        self.clear_navigation_command()
        self.publish_command_result(
            command_id=command_id,
            status=final_status,
            error_message=error_message,
        )

    def clear_navigation_command(self):
        self.navigation_in_progress = False
        self.current_command_id = None
        self.current_goal_handle = None
        self.cancel_navigation_when_accepted = False

    def process_lift_command(self, payload):
        command_id = payload.get("command_id")
        action = str(payload.get("action", "")).strip().upper()

        if command_id is None or action not in {"UP", "DOWN"}:
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                command_type="LIFT",
                error_message="Invalid LIFT payload",
            )
            return

        if not self.control_enabled:
            self.publish_command_result(
                command_id=command_id,
                status="FAILED",
                command_type=f"LIFT_{action}",
                error_message="Equipment is STOPPED",
            )
            return

        if self.lift_in_progress:
            self.publish_command_result(
                command_id=command_id,
                status="BUSY",
                command_type=f"LIFT_{action}",
                error_message="Lift command already in progress",
            )
            return

        self.lift_in_progress = True
        self.current_lift_command_id = command_id
        self.current_lift_action = action
        self.lift_command_deadline = (
            time.monotonic() + LIFT_COMMAND_TIMEOUT
        )

        message = String()
        message.data = action
        self.lift_command_publisher.publish(message)

        self.get_logger().info(
            f"LIFT command_id={command_id} "
            f"action={action} -> {ROS_LIFT_COMMAND_TOPIC}"
        )
        self.publish_command_result(
            command_id=command_id,
            status="RUNNING",
            command_type=f"LIFT_{action}",
        )

    def lift_state_callback(self, message: String):
        state = message.data.strip().upper()

        if not state:
            return

        state_changed = state != self.current_lift_state
        self.current_lift_state = state

        if state_changed:
            payload = {"lift_state": state}
            self.mqtt_client.publish(
                MQTT_LIFT_STATE_TOPIC,
                json.dumps(payload),
            )
            self.get_logger().info(f"Lift state: {state}")

        if not self.lift_in_progress:
            return

        if state != self.current_lift_action:
            return

        command_id = self.current_lift_command_id
        action = self.current_lift_action
        self.get_logger().info(
            f"LIFT SUCCEEDED: command_id={command_id} state={state}"
        )
        self.clear_lift_command()
        self.publish_command_result(
            command_id=command_id,
            status="SUCCESS",
            command_type=f"LIFT_{action}",
        )

    def check_lift_timeout(self):
        if not self.lift_in_progress:
            return

        if self.lift_command_deadline is None:
            return

        if time.monotonic() < self.lift_command_deadline:
            return

        command_id = self.current_lift_command_id
        action = self.current_lift_action
        self.get_logger().error(
            f"LIFT timeout: command_id={command_id} "
            f"action={action} last_state={self.current_lift_state}"
        )
        self.clear_lift_command()
        self.publish_command_result(
            command_id=command_id,
            status="FAILED",
            command_type=f"LIFT_{action}",
            error_message=(
                "Lift did not reach target state "
                f"within {LIFT_COMMAND_TIMEOUT:.1f}s"
            ),
        )

    def clear_lift_command(self):
        self.lift_in_progress = False
        self.current_lift_command_id = None
        self.current_lift_action = None
        self.lift_command_deadline = None

    def publish_command_result(
        self,
        command_id,
        status,
        error_message=None,
        command_type="NAVIGATE",
    ):
        payload = {
            "command_id": command_id,
            "equipment_code": EQUIPMENT_CODE,
            "command_type": command_type,
            "status": status,
        }

        if error_message is not None:
            payload["error_message"] = error_message

        self.mqtt_client.publish(
            MQTT_COMMAND_RESULT_TOPIC,
            json.dumps(payload),
            qos=1,
        )
        print(
            f"[MQTT] result={payload}",
            flush=True,
        )

    def destroy_node(self):
        self.publish_zero_velocity()
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
