#!/usr/bin/env python3

import json
import math
import os
import time
import uuid

import paho.mqtt.client as mqtt
import rclpy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

EQUIPMENT_CODE = os.getenv("EQUIPMENT_CODE", "AMR_IN")
ROS_MAP_POSE_TOPIC = os.getenv("ROS_MAP_POSE_TOPIC", "/amr_a/map_pose")
ROS_INITIAL_POSE_TOPIC = os.getenv("ROS_INITIAL_POSE_TOPIC", "/initialpose")
MAP_FRAME = os.getenv("MAP_FRAME", "map")
BASE_FRAME = os.getenv("BASE_FRAME", "base_link")

POSE_PUBLISH_INTERVAL = float(os.getenv("POSE_PUBLISH_INTERVAL", "0.5"))
POSE_TIMEOUT = float(os.getenv("POSE_TIMEOUT", "2.0"))
POSITION_TOLERANCE_M = float(os.getenv("POSITION_TOLERANCE_M", "0.05"))
YAW_TOLERANCE_RAD = math.radians(
    float(os.getenv("YAW_TOLERANCE_DEG", "2.0"))
)
INITIAL_POSE_RETRY_COUNT = int(
    os.getenv("INITIAL_POSE_RETRY_COUNT", "5")
)
INITIAL_POSE_RETRY_INTERVAL = float(
    os.getenv("INITIAL_POSE_RETRY_INTERVAL", "0.5")
)

MQTT_POSE_TOPIC = f"controltower/amr/{EQUIPMENT_CODE}/pose"
MQTT_SYNC_TOPIC = f"controltower/amr/{EQUIPMENT_CODE}/pose_sync"


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class PoseSyncManager(Node):
    """Own the Control Tower canonical map pose for one AMR.

    Step 1-5 scope:
    - Consume the actual Isaac World pose published as frame=map.
    - Publish only that pose as the canonical MQTT/DB position.
    - Compare it with map->base_link when Nav2 TF exists.
    - Report OFFLINE/SYNCING/SYNCED/DESYNC.
    - Initialize AMCL from the current canonical pose whenever Nav2 appears.

    Isaac pose restore is intentionally left for a later step.
    """

    def __init__(self):
        super().__init__("control_tower_pose_sync_manager")

        self.session_epoch_ms = time.time_ns() // 1_000_000
        self.session_id = (
            f"{self.session_epoch_ms}-"
            f"{uuid.uuid4().hex[:8]}"
        )
        self.pose_seq = 0

        self.latest_pose = None
        self.last_pose_rx_time = None
        self.last_pose_publish_time = 0.0
        self.last_sync_status = None
        self.nav2_initial_pose_subscriber_available = False
        self.initial_pose_retries_remaining = 0
        self.next_initial_pose_publish_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.create_subscription(
            PoseStamped,
            ROS_MAP_POSE_TOPIC,
            self.map_pose_callback,
            10,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            ROS_INITIAL_POSE_TOPIC,
            10,
        )

        self.mqtt_client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"pose-sync-{EQUIPMENT_CODE.lower()}",
        )
        self.mqtt_client.on_connect = self.on_mqtt_connect

        offline_payload = {
            "status": "OFFLINE",
            "session_id": self.session_id,
            "session_epoch_ms": self.session_epoch_ms,
        }
        self.mqtt_client.will_set(
            MQTT_SYNC_TOPIC,
            json.dumps(offline_payload),
            qos=1,
            retain=True,
        )

        self.get_logger().info(
            f"Connecting MQTT broker: {MQTT_HOST}:{MQTT_PORT}"
        )
        self.mqtt_client.connect(
            MQTT_HOST,
            MQTT_PORT,
            keepalive=60,
        )
        self.mqtt_client.loop_start()

        self.create_timer(0.2, self.process_pose)
        self.create_timer(0.2, self.manage_nav2_initial_pose)
        self.create_timer(0.5, self.check_sync)

        self.get_logger().info(
            f"Canonical pose input: {ROS_MAP_POSE_TOPIC}"
        )
        self.get_logger().info(
            f"Canonical MQTT pose: {MQTT_POSE_TOPIC}"
        )
        self.get_logger().info(
            f"Pose sync MQTT: {MQTT_SYNC_TOPIC}"
        )
        self.get_logger().info(
            f"TF check: {MAP_FRAME} -> {BASE_FRAME}"
        )
        self.get_logger().info(
            f"Nav2 initial pose output: {ROS_INITIAL_POSE_TOPIC}"
        )
        self.get_logger().info(
            f"Tolerance: position={POSITION_TOLERANCE_M:.3f} m "
            f"yaw={math.degrees(YAW_TOLERANCE_RAD):.1f} deg"
        )

    def on_mqtt_connect(
        self,
        client,
        userdata,
        flags,
        reason_code,
        properties,
    ):
        print(
            f"[POSE SYNC][MQTT] Connected reason_code={reason_code}",
            flush=True,
        )

    def map_pose_callback(self, message: PoseStamped):
        if message.header.frame_id != MAP_FRAME:
            self.get_logger().error(
                f"Rejected non-map pose frame={message.header.frame_id!r}"
            )
            return

        position = message.pose.position
        orientation = message.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self.latest_pose = (
            float(position.x),
            float(position.y),
            float(yaw),
        )
        self.last_pose_rx_time = time.monotonic()

    def manage_nav2_initial_pose(self):
        subscriber_count = (
            self.initial_pose_publisher.get_subscription_count()
        )
        subscriber_available = subscriber_count > 0

        if (
            subscriber_available
            and not self.nav2_initial_pose_subscriber_available
        ):
            self.initial_pose_retries_remaining = (
                INITIAL_POSE_RETRY_COUNT
            )
            self.next_initial_pose_publish_time = 0.0
            self.get_logger().info(
                "Nav2 /initialpose subscriber detected; "
                "scheduling canonical pose initialization"
            )
        elif (
            not subscriber_available
            and self.nav2_initial_pose_subscriber_available
        ):
            self.initial_pose_retries_remaining = 0
            self.get_logger().warning(
                "Nav2 /initialpose subscriber disappeared; "
                "waiting for restart"
            )

        self.nav2_initial_pose_subscriber_available = (
            subscriber_available
        )

        if not subscriber_available or self.latest_pose is None:
            return

        if self.initial_pose_retries_remaining <= 0:
            return

        now = time.monotonic()
        if now < self.next_initial_pose_publish_time:
            return

        self.publish_initial_pose()
        self.initial_pose_retries_remaining -= 1
        self.next_initial_pose_publish_time = (
            now + INITIAL_POSE_RETRY_INTERVAL
        )

    def publish_initial_pose(self):
        x, y, yaw = self.latest_pose
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = MAP_FRAME
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # AMCL's conventional planar initial-pose covariance:
        # x/y variance 0.25 m^2 and yaw variance (15 deg)^2.
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = math.radians(15.0) ** 2

        self.initial_pose_publisher.publish(message)
        attempt = (
            INITIAL_POSE_RETRY_COUNT
            - self.initial_pose_retries_remaining
            + 1
        )
        self.get_logger().info(
            f"Published Nav2 initial pose attempt={attempt}/"
            f"{INITIAL_POSE_RETRY_COUNT} "
            f"x={x:.3f} y={y:.3f} yaw={yaw:.3f}"
        )

    def process_pose(self):
        if self.latest_pose is None:
            return

        now = time.monotonic()
        if now - self.last_pose_publish_time < POSE_PUBLISH_INTERVAL:
            return

        self.last_pose_publish_time = now
        self.pose_seq += 1

        x, y, yaw = self.latest_pose
        payload = {
            "equipment_code": EQUIPMENT_CODE,
            "frame": MAP_FRAME,
            "x": x,
            "y": y,
            "yaw": yaw,
            "source": "ISAAC_WORLD",
            "seq": self.pose_seq,
            "session_id": self.session_id,
            "session_epoch_ms": self.session_epoch_ms,
        }

        result = self.mqtt_client.publish(
            MQTT_POSE_TOPIC,
            json.dumps(payload),
            qos=1,
            retain=False,
        )

        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(
                f"MQTT pose publish failed rc={result.rc}"
            )
            return

        self.get_logger().info(
            f"Canonical map pose seq={self.pose_seq} "
            f"x={x:.3f} y={y:.3f} yaw={yaw:.3f}"
        )

    def check_sync(self):
        if (
            self.latest_pose is None
            or self.last_pose_rx_time is None
            or time.monotonic() - self.last_pose_rx_time > POSE_TIMEOUT
        ):
            self.publish_sync_status("OFFLINE")
            return

        if (
            not self.nav2_initial_pose_subscriber_available
            or self.initial_pose_retries_remaining > 0
        ):
            self.publish_sync_status("SYNCING")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                Time(),
            )
        except Exception:
            self.publish_sync_status("SYNCING")
            return

        x, y, yaw = self.latest_pose

        tf_position = transform.transform.translation
        tf_orientation = transform.transform.rotation
        tf_yaw = quaternion_to_yaw(
            tf_orientation.x,
            tf_orientation.y,
            tf_orientation.z,
            tf_orientation.w,
        )

        position_error = math.hypot(
            x - float(tf_position.x),
            y - float(tf_position.y),
        )
        yaw_error = abs(angle_error(yaw, tf_yaw))

        if (
            position_error <= POSITION_TOLERANCE_M
            and yaw_error <= YAW_TOLERANCE_RAD
        ):
            status = "SYNCED"
        else:
            status = "DESYNC"

        self.publish_sync_status(
            status,
            position_error=position_error,
            yaw_error=yaw_error,
        )

    def publish_sync_status(
        self,
        status,
        position_error=None,
        yaw_error=None,
    ):
        if status == self.last_sync_status:
            return

        self.last_sync_status = status

        payload = {
            "status": status,
            "session_id": self.session_id,
            "session_epoch_ms": self.session_epoch_ms,
        }

        if position_error is not None:
            payload["position_error_m"] = float(position_error)

        if yaw_error is not None:
            payload["yaw_error_rad"] = float(yaw_error)

        self.mqtt_client.publish(
            MQTT_SYNC_TOPIC,
            json.dumps(payload),
            qos=1,
            retain=True,
        )

        self.get_logger().info(
            f"Pose sync status -> {status}"
        )

    def shutdown(self):
        payload = {
            "status": "OFFLINE",
            "session_id": self.session_id,
            "session_epoch_ms": self.session_epoch_ms,
        }
        self.mqtt_client.publish(
            MQTT_SYNC_TOPIC,
            json.dumps(payload),
            qos=1,
            retain=True,
        )
        time.sleep(0.05)
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


def main():
    rclpy.init()
    node = PoseSyncManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
