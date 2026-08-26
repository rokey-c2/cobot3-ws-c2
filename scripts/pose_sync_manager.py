#!/usr/bin/env python3

import json
import math
import os
import queue
import time
import uuid

import paho.mqtt.client as mqtt
import rclpy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

EQUIPMENT_CODE = os.getenv("EQUIPMENT_CODE", "AMR_IN")
ROS_MAP_POSE_TOPIC = os.getenv("ROS_MAP_POSE_TOPIC", "/amr_a/map_pose")
ROS_INITIAL_POSE_TOPIC = os.getenv("ROS_INITIAL_POSE_TOPIC", "/initialpose")
ROS_RESTORE_POSE_TOPIC = os.getenv(
    "ROS_RESTORE_POSE_TOPIC",
    "/amr_a/restore_pose",
)
ROS_POSE_SOURCE_SESSION_TOPIC = os.getenv(
    "ROS_POSE_SOURCE_SESSION_TOPIC",
    "/amr_a/pose_source_session",
)
MAP_FRAME = os.getenv("MAP_FRAME", "map")
BASE_FRAME = os.getenv("BASE_FRAME", "base_link")

POSE_PUBLISH_INTERVAL = float(os.getenv("POSE_PUBLISH_INTERVAL", "0.5"))
POSE_TIMEOUT = float(os.getenv("POSE_TIMEOUT", "2.0"))
POSITION_TOLERANCE_M = float(os.getenv("POSITION_TOLERANCE_M", "0.05"))
YAW_TOLERANCE_RAD = math.radians(
    float(os.getenv("YAW_TOLERANCE_DEG", "2.0"))
)
DESYNC_DEBOUNCE_SECONDS = max(
    0.0,
    float(os.getenv("DESYNC_DEBOUNCE_SECONDS", "2.0")),
)
INITIAL_POSE_RETRY_COUNT = int(
    os.getenv("INITIAL_POSE_RETRY_COUNT", "5")
)
INITIAL_POSE_RETRY_INTERVAL = float(
    os.getenv("INITIAL_POSE_RETRY_INTERVAL", "0.5")
)
RESTORE_REQUEST_INTERVAL = float(
    os.getenv("RESTORE_REQUEST_INTERVAL", "1.0")
)
RESTORE_PUBLISH_INTERVAL = float(
    os.getenv("RESTORE_PUBLISH_INTERVAL", "0.5")
)
RESTORE_MAX_ATTEMPTS = int(
    os.getenv("RESTORE_MAX_ATTEMPTS", "20")
)
RESTORE_VERIFY_SAMPLES = int(
    os.getenv("RESTORE_VERIFY_SAMPLES", "3")
)
RESTORE_POSITION_TOLERANCE_M = float(
    os.getenv("RESTORE_POSITION_TOLERANCE_M", "0.05")
)
RESTORE_YAW_TOLERANCE_RAD = math.radians(
    float(os.getenv("RESTORE_YAW_TOLERANCE_DEG", "2.0"))
)

MQTT_POSE_TOPIC = f"controltower/amr/{EQUIPMENT_CODE}/pose"
MQTT_SYNC_TOPIC = f"controltower/amr/{EQUIPMENT_CODE}/pose_sync"
MQTT_RESTORE_REQUEST_TOPIC = (
    f"controltower/amr/{EQUIPMENT_CODE}/pose_restore/request"
)
MQTT_RESTORE_RESPONSE_TOPIC = (
    f"controltower/amr/{EQUIPMENT_CODE}/pose_restore/response"
)

RESTORE_WAITING_FOR_ISAAC = "WAITING_FOR_ISAAC"
RESTORE_WAITING_FOR_DB = "WAITING_FOR_DB"
RESTORE_APPLYING = "APPLYING"
RESTORE_READY = "READY"
RESTORE_FAILED = "FAILED"


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose_source_session_qos():
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class PoseSyncManager(Node):
    """Own canonical pose persistence, Isaac restore, and Nav2 alignment."""

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
        self.desync_started_at = None

        self.isaac_session_id = None
        self.restore_state = RESTORE_WAITING_FOR_ISAAC
        self.restore_request_id = None
        self.restore_target = None
        self.restore_attempts = 0
        self.restore_verified_samples = 0
        self.last_restore_request_time = 0.0
        self.last_restore_publish_time = 0.0
        self.restore_response_queue = queue.Queue()

        self.nav2_initial_pose_subscriber_available = False
        self.nav2_needs_initial_pose = True
        self.initial_pose_retries_remaining = 0
        self.next_initial_pose_publish_time = 0.0

        self.mqtt_connected = False

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
        self.create_subscription(
            String,
            ROS_POSE_SOURCE_SESSION_TOPIC,
            self.pose_source_session_callback,
            pose_source_session_qos(),
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            ROS_INITIAL_POSE_TOPIC,
            10,
        )
        self.restore_pose_publisher = self.create_publisher(
            PoseStamped,
            ROS_RESTORE_POSE_TOPIC,
            10,
        )

        self.mqtt_client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"pose-sync-{EQUIPMENT_CODE.lower()}",
        )
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message

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

        self.create_timer(0.2, self.manage_restore)
        self.create_timer(0.2, self.process_pose)
        self.create_timer(0.2, self.manage_nav2_initial_pose)
        self.create_timer(0.5, self.check_sync)

        self.get_logger().info(
            f"Canonical pose input: {ROS_MAP_POSE_TOPIC}"
        )
        self.get_logger().info(
            f"Isaac session input: {ROS_POSE_SOURCE_SESSION_TOPIC}"
        )
        self.get_logger().info(
            f"Isaac restore output: {ROS_RESTORE_POSE_TOPIC}"
        )
        self.get_logger().info(
            f"Canonical MQTT pose: {MQTT_POSE_TOPIC}"
        )
        self.get_logger().info(
            f"DB restore MQTT: {MQTT_RESTORE_REQUEST_TOPIC}"
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
        self.get_logger().info(
            f"DESYNC debounce: {DESYNC_DEBOUNCE_SECONDS:.1f} s "
            "of continuous mismatch"
        )

    def on_mqtt_connect(
        self,
        client,
        userdata,
        flags,
        reason_code,
        properties,
    ):
        self.mqtt_connected = True
        client.subscribe(MQTT_RESTORE_RESPONSE_TOPIC, qos=1)
        print(
            f"[POSE SYNC][MQTT] Connected reason_code={reason_code}",
            flush=True,
        )
        print(
            f"[POSE SYNC][MQTT] Subscribed: "
            f"{MQTT_RESTORE_RESPONSE_TOPIC}",
            flush=True,
        )

    def on_mqtt_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
    ):
        del client, userdata, disconnect_flags, properties
        self.mqtt_connected = False
        print(
            f"[POSE SYNC][MQTT] Disconnected reason_code={reason_code}",
            flush=True,
        )

    def on_mqtt_message(self, client, userdata, message):
        del client, userdata

        if message.topic != MQTT_RESTORE_RESPONSE_TOPIC:
            return

        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.get_logger().error(
                "Invalid DB restore response JSON"
            )
            return

        self.restore_response_queue.put(payload)

    def pose_source_session_callback(self, message: String):
        isaac_session_id = message.data.strip()

        if not isaac_session_id:
            self.get_logger().error("Rejected empty Isaac pose session")
            return

        if isaac_session_id == self.isaac_session_id:
            return

        previous_session = self.isaac_session_id
        self.isaac_session_id = isaac_session_id
        self.latest_pose = None
        self.last_pose_rx_time = None
        self.desync_started_at = None
        self.restore_state = RESTORE_WAITING_FOR_DB
        self.restore_request_id = uuid.uuid4().hex
        self.restore_target = None
        self.restore_attempts = 0
        self.restore_verified_samples = 0
        self.last_restore_request_time = 0.0
        self.last_restore_publish_time = 0.0
        self.initial_pose_retries_remaining = 0
        self.nav2_needs_initial_pose = True

        self.get_logger().warning(
            "Isaac pose source session changed: "
            f"{previous_session!r} -> {isaac_session_id!r}; "
            "canonical DB writes and Nav2 initialization are locked"
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
        pose = (
            float(position.x),
            float(position.y),
            float(yaw),
        )

        if not all(math.isfinite(value) for value in pose):
            self.get_logger().error("Rejected non-finite Isaac map pose")
            return

        self.latest_pose = pose
        self.last_pose_rx_time = time.monotonic()

        if (
            self.restore_state == RESTORE_APPLYING
            and self.restore_attempts > 0
            and self.restore_target is not None
        ):
            self.verify_restored_pose()

    def manage_restore(self):
        self.process_restore_responses()

        if self.restore_state == RESTORE_WAITING_FOR_DB:
            self.request_saved_pose_when_ready()
            return

        if self.restore_state != RESTORE_APPLYING:
            return

        if self.restore_pose_publisher.get_subscription_count() < 1:
            return

        if self.restore_attempts >= RESTORE_MAX_ATTEMPTS:
            self.restore_state = RESTORE_FAILED
            self.get_logger().error(
                "Isaac pose restore failed verification after "
                f"{RESTORE_MAX_ATTEMPTS} attempts; canonical writes remain locked"
            )
            return

        now = time.monotonic()
        if now - self.last_restore_publish_time < RESTORE_PUBLISH_INTERVAL:
            return

        self.last_restore_publish_time = now
        self.publish_restore_pose()

    def process_restore_responses(self):
        while not self.restore_response_queue.empty():
            payload = self.restore_response_queue.get()
            self.handle_restore_response(payload)

    def request_saved_pose_when_ready(self):
        if not self.mqtt_connected or self.isaac_session_id is None:
            return

        now = time.monotonic()
        if now - self.last_restore_request_time < RESTORE_REQUEST_INTERVAL:
            return

        self.last_restore_request_time = now
        payload = {
            "request_id": self.restore_request_id,
            "equipment_code": EQUIPMENT_CODE,
            "isaac_session_id": self.isaac_session_id,
            "manager_session_id": self.session_id,
            "manager_session_epoch_ms": self.session_epoch_ms,
        }
        result = self.mqtt_client.publish(
            MQTT_RESTORE_REQUEST_TOPIC,
            json.dumps(payload),
            qos=1,
            retain=False,
        )

        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().info(
                "Requested last canonical pose from DB: "
                f"request_id={self.restore_request_id}"
            )
        else:
            self.get_logger().error(
                f"DB restore request publish failed rc={result.rc}"
            )

    def handle_restore_response(self, payload):
        if self.restore_state != RESTORE_WAITING_FOR_DB:
            return

        if str(payload.get("request_id", "")).strip() != self.restore_request_id:
            return

        if (
            str(payload.get("isaac_session_id", "")).strip()
            != self.isaac_session_id
        ):
            return

        status = str(payload.get("status", "")).strip().upper()

        if status == "NOT_FOUND":
            self.complete_restore(
                "DB has no saved canonical pose; using current Isaac pose"
            )
            return

        if status == "ERROR":
            self.get_logger().error(
                "DB restore lookup error: "
                f"{payload.get('error', 'unknown error')}"
            )
            self.last_restore_request_time = 0.0
            return

        if status != "FOUND" or payload.get("frame") != MAP_FRAME:
            self.get_logger().error(
                f"Rejected invalid DB restore response: {payload}"
            )
            return

        try:
            target = (
                float(payload["x"]),
                float(payload["y"]),
                float(payload["yaw"]),
            )
        except (KeyError, TypeError, ValueError):
            self.get_logger().error(
                f"Rejected malformed DB restore response: {payload}"
            )
            return

        if not all(math.isfinite(value) for value in target):
            self.get_logger().error(
                "Rejected non-finite DB restore target"
            )
            return

        self.restore_target = target
        self.restore_attempts = 0
        self.restore_verified_samples = 0
        self.last_restore_publish_time = 0.0
        self.restore_state = RESTORE_APPLYING

        self.get_logger().warning(
            "DB canonical pose received; restoring Isaac: "
            f"x={target[0]:.3f} y={target[1]:.3f} "
            f"yaw={target[2]:.3f}"
        )

    def publish_restore_pose(self):
        x, y, yaw = self.restore_target
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = MAP_FRAME
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = 0.0
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)

        self.restore_pose_publisher.publish(message)
        self.restore_attempts += 1

        self.get_logger().info(
            f"Published Isaac restore pose attempt={self.restore_attempts}/"
            f"{RESTORE_MAX_ATTEMPTS} "
            f"x={x:.3f} y={y:.3f} yaw={yaw:.3f}"
        )

    def verify_restored_pose(self):
        x, y, yaw = self.latest_pose
        target_x, target_y, target_yaw = self.restore_target
        position_error = math.hypot(x - target_x, y - target_y)
        yaw_error = abs(angle_error(yaw, target_yaw))

        if (
            position_error <= RESTORE_POSITION_TOLERANCE_M
            and yaw_error <= RESTORE_YAW_TOLERANCE_RAD
        ):
            self.restore_verified_samples += 1
        else:
            self.restore_verified_samples = 0
            return

        if self.restore_verified_samples < RESTORE_VERIFY_SAMPLES:
            return

        self.complete_restore(
            "Isaac World pose verified after DB restore: "
            f"position_error={position_error:.4f} m "
            f"yaw_error={math.degrees(yaw_error):.2f} deg"
        )

    def complete_restore(self, reason):
        self.restore_state = RESTORE_READY
        self.initial_pose_retries_remaining = 0
        self.nav2_needs_initial_pose = True
        self.get_logger().info(f"Pose restore ready: {reason}")

    def restore_is_ready(self):
        return self.restore_state == RESTORE_READY

    def manage_nav2_initial_pose(self):
        subscriber_count = (
            self.initial_pose_publisher.get_subscription_count()
        )
        subscriber_available = subscriber_count > 0

        if (
            subscriber_available
            and not self.nav2_initial_pose_subscriber_available
        ):
            self.nav2_needs_initial_pose = True
            self.get_logger().info(
                "Nav2 /initialpose subscriber detected"
            )
        elif (
            not subscriber_available
            and self.nav2_initial_pose_subscriber_available
        ):
            self.initial_pose_retries_remaining = 0
            self.nav2_needs_initial_pose = True
            self.get_logger().warning(
                "Nav2 /initialpose subscriber disappeared; "
                "waiting for restart"
            )

        self.nav2_initial_pose_subscriber_available = (
            subscriber_available
        )

        if (
            not subscriber_available
            or not self.restore_is_ready()
            or self.latest_pose is None
        ):
            return

        if (
            self.nav2_needs_initial_pose
            and self.initial_pose_retries_remaining <= 0
        ):
            self.initial_pose_retries_remaining = (
                INITIAL_POSE_RETRY_COUNT
            )
            self.next_initial_pose_publish_time = 0.0
            self.nav2_needs_initial_pose = False
            self.get_logger().info(
                "Scheduling Nav2 initialization from verified Isaac pose"
            )

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
        if self.latest_pose is None or not self.restore_is_ready():
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
        now = time.monotonic()

        if (
            self.latest_pose is None
            or self.last_pose_rx_time is None
            or now - self.last_pose_rx_time > POSE_TIMEOUT
        ):
            self.desync_started_at = None
            self.publish_sync_status("OFFLINE")
            return

        if self.restore_state == RESTORE_FAILED:
            self.desync_started_at = None
            self.publish_sync_status("DESYNC")
            return

        if not self.restore_is_ready():
            self.desync_started_at = None
            self.publish_sync_status("SYNCING")
            return

        if (
            not self.nav2_initial_pose_subscriber_available
            or self.nav2_needs_initial_pose
            or self.initial_pose_retries_remaining > 0
        ):
            self.desync_started_at = None
            self.publish_sync_status("SYNCING")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                Time(),
            )
        except Exception:
            self.desync_started_at = None
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
        pose_matches_tf = (
            position_error <= POSITION_TOLERANCE_M
            and yaw_error <= YAW_TOLERANCE_RAD
        )

        if pose_matches_tf:
            self.desync_started_at = None
            status = "SYNCED"
        else:
            if self.desync_started_at is None:
                self.desync_started_at = now

            mismatch_duration = now - self.desync_started_at
            if mismatch_duration < DESYNC_DEBOUNCE_SECONDS:
                # Keep a previously confirmed SYNCED state during a short,
                # moving-pose/TF timing gap. During startup, remain SYNCING
                # until a matching sample is observed.
                if self.last_sync_status != "SYNCED":
                    self.publish_sync_status(
                        "SYNCING",
                        position_error=position_error,
                        yaw_error=yaw_error,
                    )
                return

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
        self.get_logger().info(f"Pose sync status -> {status}")

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

