"""ROS 2 box detector and web video stream for the P3020 camera.

Subscribes:
    /arm_a/rgb                          (sensor_msgs/msg/Image)

Publishes:
    /box_pixel                          (geometry_msgs/msg/PointStamped)
    /box_pixel/result                   (std_msgs/String, JSON: stamp_ns/detection)
    /p3020/vision/image_annotated       (sensor_msgs/msg/Image)

Subscribes:
    /box_pixel/validated                (std_msgs/msg/String, mission-validated HUD target)

Web stream:
    http://<vision-pc-ip>:8091/stream.mjpg

The raw camera topic is left unchanged.  The annotated topic and MJPEG stream
contain a green laser HUD drawn around the highest-confidence parcel box.
"""

import json
import math
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "..", ".."))
sys.path.insert(0, _THIS_DIR)

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from object_detector import ObjectDetector

MODEL_PATH = os.path.join(_REPO_ROOT, "models", "parcel_box_yolo_model", "best.onnx")
IMAGE_TOPIC = "/arm_a/rgb"
BOX_PIXEL_TOPIC = "/box_pixel"
ANNOTATED_IMAGE_TOPIC = "/p3020/vision/image_annotated"
STREAM_HOST = "0.0.0.0"
STREAM_PORT = 8091
STREAM_PATH = "/stream.mjpg"

_ENCODING_CHANNELS = {"rgb8": 3, "bgr8": 3}
_LASER_GREEN = (0, 255, 92)
_LASER_GREEN_DIM = (0, 150, 54)


class _MjpegServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _MjpegHandler(BaseHTTPRequestHandler):
    """Serve the latest annotated frame without introducing web dependencies."""

    server_version = "P3020Vision/1.0"

    def do_GET(self):
        source = self.server.frame_source

        if self.path.rstrip("/") == "/health":
            payload = json.dumps(
                source.stream_health()
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(payload)
            return

        if self.path.split("?", 1)[0] != source.stream_path:
            self.send_error(404, "P3020 stream not found")
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_sequence = -1
        try:
            self.wfile.write(b"--frame\r\n")
            while rclpy.ok():
                jpeg, sequence = source.wait_for_stream_frame(last_sequence, timeout=2.0)
                if jpeg is None or sequence == last_sequence:
                    continue
                last_sequence = sequence
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                # Complete the multipart boundary now. Otherwise browsers can
                # hold a newly validated HUD until the next camera frame.
                self.wfile.write(b"\r\n--frame\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def log_message(self, _format, *_args):
        # Browser reconnects are expected; keep the ROS log readable.
        return


class BoxDetectorNode(Node):
    def __init__(self):
        super().__init__("p3020_box_detector")
        cv2.setNumThreads(1)

        self.declare_parameter("image_topic", IMAGE_TOPIC)
        self.declare_parameter("stream_image_topic", "")
        self.declare_parameter("box_pixel_topic", BOX_PIXEL_TOPIC)
        self.declare_parameter("annotated_image_topic", ANNOTATED_IMAGE_TOPIC)
        self.declare_parameter("model_path", MODEL_PATH)
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("stream_host", STREAM_HOST)
        self.declare_parameter("stream_port", STREAM_PORT)
        self.declare_parameter("stream_path", STREAM_PATH)
        self.declare_parameter("jpeg_quality", 82)
        # Optional image-space exclusion for fixed background false positives.
        # Disabled by default; configured only for the P3020 IN camera.
        self.declare_parameter("ignore_region_x", -1.0)
        self.declare_parameter("ignore_region_y", -1.0)
        self.declare_parameter("ignore_region_radius", 0.0)

        image_topic = str(self.get_parameter("image_topic").value)
        pixel_topic = str(self.get_parameter("box_pixel_topic").value)
        annotated_topic = str(self.get_parameter("annotated_image_topic").value)
        model_path = str(self.get_parameter("model_path").value)
        conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.stream_host = str(self.get_parameter("stream_host").value)
        self.stream_port = int(self.get_parameter("stream_port").value)
        self.stream_path = self._normalise_stream_path(
            str(self.get_parameter("stream_path").value)
        )
        self.jpeg_quality = int(np.clip(self.get_parameter("jpeg_quality").value, 40, 95))
        self.ignore_region_x = float(self.get_parameter("ignore_region_x").value)
        self.ignore_region_y = float(self.get_parameter("ignore_region_y").value)
        self.ignore_region_radius = max(
            0.0, float(self.get_parameter("ignore_region_radius").value)
        )

        self.detector = ObjectDetector(model_path, conf_threshold=conf_threshold)
        # Stream incoming frames immediately; run CPU-bound inference separately.
        self._inference_queue = queue.Queue(maxsize=1)
        self._detection_lock = threading.Lock()
        self._latest_detection = None
        self._latest_detection_time = 0.0
        self._latest_source = None
        self._latest_source_time = 0.0
        self._inference_thread = threading.Thread(
            target=self._inference_worker, name="p3020-yolo", daemon=True
        )
        self._inference_thread.start()
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.pixel_pub = self.create_publisher(PointStamped, pixel_topic, 10)
        self.result_pub = self.create_publisher(String, pixel_topic + "/result", 10)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        # Raw YOLO candidates are intentionally not shown in the HUD. The
        # mission agent publishes a target here only after depth/world checks.
        self.validated_sub = self.create_subscription(
            String, pixel_topic + "/validated", self._on_validated_detection, 10
        )

        self._frame_condition = threading.Condition()
        self._latest_jpeg = None
        self._frame_sequence = 0
        self._stream_server = None
        self._stream_thread = None
        self._start_stream_server()
        stream_topic = str(self.get_parameter("stream_image_topic").value)
        self.preview_sub = (
            self.create_subscription(Image, stream_topic, self.preview_callback, qos_profile_sensor_data)
            if stream_topic else None
        )

        self.get_logger().info(
            f"listening: {image_topic} | detection: {pixel_topic} | "
            f"annotated: {annotated_topic}"
        )

    @staticmethod
    def _normalise_stream_path(path):
        path = path.strip() or STREAM_PATH
        return path if path.startswith("/") else f"/{path}"

    @property
    def has_stream_frame(self):
        with self._frame_condition:
            return self._latest_jpeg is not None

    def stream_health(self):
        now = time.monotonic()
        with self._detection_lock:
            frame_age = now - self._latest_source_time if self._latest_source is not None else None
            detection = self._latest_detection if now - self._latest_detection_time <= 1.0 else None
        live = frame_age is not None and frame_age <= 3.0
        return {
            "status": "ok",
            "stream": self.stream_path,
            "frame_ready": self.has_stream_frame,
            "frame_age_seconds": frame_age,
            "live": live,
            "detection_state": "TARGET LOCKED" if live and detection is not None else "SCANNING" if live else "WAITING",
            "confidence": detection["conf"] if live and detection is not None else None,
        }

    def wait_for_stream_frame(self, last_sequence, timeout):
        with self._frame_condition:
            if self._frame_sequence == last_sequence:
                self._frame_condition.wait(timeout=timeout)
            return self._latest_jpeg, self._frame_sequence

    def _start_stream_server(self):
        try:
            self._stream_server = _MjpegServer(
                (self.stream_host, self.stream_port), _MjpegHandler
            )
            self._stream_server.frame_source = self
            self._stream_thread = threading.Thread(
                target=self._stream_server.serve_forever,
                name="p3020-mjpeg",
                daemon=True,
            )
            self._stream_thread.start()
            self.get_logger().info(
                f"web stream: http://{self.stream_host}:{self.stream_port}{self.stream_path}"
            )
        except OSError as error:
            self._stream_server = None
            self.get_logger().error(
                f"MJPEG server could not bind {self.stream_host}:{self.stream_port}: {error}"
            )

    @staticmethod
    def imgmsg_to_rgb(msg: Image):
        channels = _ENCODING_CHANNELS.get(msg.encoding)
        if channels is None:
            return None

        row_size = msg.width * channels
        if msg.step < row_size:
            return None

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        required = msg.height * msg.step
        if arr.size < required:
            return None

        rows = arr[:required].reshape((msg.height, msg.step))
        image = rows[:, :row_size].reshape((msg.height, msg.width, channels))
        if msg.encoding == "bgr8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    def image_callback(self, msg: Image):
        rgb = self.imgmsg_to_rgb(msg)
        if rgb is None:
            self.get_logger().warn(
                f'unsupported or truncated image (encoding="{msg.encoding}")',
                throttle_duration_sec=5.0,
            )
            return

        received_at = time.monotonic()
        with self._detection_lock:
            self._latest_source = (msg, rgb)
            self._latest_source_time = received_at
            detection = self._latest_detection
            if received_at - self._latest_detection_time > 1.0:
                detection = None
        annotated_rgb = self.draw_laser_hud(rgb, detection, received_at)
        self._update_web_stream(annotated_rgb)
        self._publish_annotated(msg, annotated_rgb)

        try:
            self._inference_queue.put_nowait((msg, rgb))
        except queue.Full:
            try:
                self._inference_queue.get_nowait()
            except queue.Empty:
                pass
            self._inference_queue.put_nowait((msg, rgb))

    def preview_callback(self, msg: Image):
        """Keep web video live without displacing an exact-frame inference request."""
        rgb = self.imgmsg_to_rgb(msg)
        if rgb is None:
            return
        now = time.monotonic()
        with self._detection_lock:
            self._latest_source = (msg, rgb)
            self._latest_source_time = now
            detection = self._latest_detection if now - self._latest_detection_time <= 1.0 else None
        annotated = self.draw_laser_hud(rgb, detection, now)
        self._update_web_stream(annotated)
        self._publish_annotated(msg, annotated)

    def _on_validated_detection(self, msg: String):
        """Show only a candidate accepted by P3020's depth/world validation."""
        try:
            payload = json.loads(msg.data)
            detection = payload.get("detection")
            if detection is not None:
                keys = ("cx", "cy", "w", "h", "conf")
                detection = {key: float(detection[key]) for key in keys}
                if not all(np.isfinite(value) for value in detection.values()):
                    return
                if detection["w"] <= 0 or detection["h"] <= 0:
                    return
        except (json.JSONDecodeError, TypeError, ValueError, KeyError):
            self.get_logger().warning("invalid validated HUD detection payload")
            return

        with self._detection_lock:
            self._latest_detection = detection
            self._latest_detection_time = time.monotonic()
            source = self._latest_source
        # Validation can arrive after the mission has stopped requesting RGB.
        # Show it immediately instead of waiting for the next scan/move frame.
        if source is not None:
            msg, rgb = source
            annotated = self.draw_laser_hud(rgb, detection, time.monotonic())
            self._update_web_stream(annotated)
            self._publish_annotated(msg, annotated)

    def _inference_worker(self):
        while rclpy.ok():
            item = self._inference_queue.get()
            if item is None:
                return
            msg, rgb = item
            try:
                candidates = self.detector.detect_candidates(rgb)
            except Exception as error:
                self.get_logger().error(f"YOLO inference failed: {error}")
                continue

            # P3020 IN repeatedly sees the same floor/AMR-edge patch at about
            # (354, 435), corresponding to the known world false positive
            # (2.265, -1.396, z~=0.31m). Suppress it before publishing either
            # the HUD target or detection results, so it cannot look locked
            # in the frontend or keep the empty-cargo scan alive. OUT leaves
            # these parameters disabled and is unaffected.
            ignore_radius = float(getattr(self, "ignore_region_radius", 0.0))
            ignore_x = float(getattr(self, "ignore_region_x", -1.0))
            ignore_y = float(getattr(self, "ignore_region_y", -1.0))
            if ignore_radius > 0.0 and ignore_x >= 0.0 and ignore_y >= 0.0:
                radius_sq = ignore_radius ** 2
                filtered = [
                    candidate for candidate in candidates
                    if ((candidate["cx"] - ignore_x) ** 2
                        + (candidate["cy"] - ignore_y) ** 2) > radius_sq
                ]
                ignored_count = len(candidates) - len(filtered)
                if ignored_count:
                    self.get_logger().warning(
                        f"ignored {ignored_count} candidate(s) in configured floor false-positive region",
                        throttle_duration_sec=5.0,
                    )
                candidates = filtered

            # An explicit negative result proves inference ran on this exact image.
            # Silence (missing camera/detector) must never mean "empty cargo".
            result = String()
            result.data = json.dumps({
                "stamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                "candidates": candidates,
                "detection": candidates[0] if candidates else None,
            })
            self.result_pub.publish(result)
            detection = candidates[0] if candidates else None
            if candidates:
                stamped = PointStamped()
                stamped.header.stamp = msg.header.stamp
                stamped.header.frame_id = msg.header.frame_id
                stamped.point.x = detection["cx"]
                stamped.point.y = detection["cy"]
                stamped.point.z = detection["conf"]
                self.pixel_pub.publish(stamped)
            # Raw model results drive the mission validator, but never the HUD.

    @staticmethod
    def draw_laser_hud(rgb, detection, now):
        """Return a copy of ``rgb`` with an animated green targeting HUD."""
        frame = rgb.copy()
        height, width = frame.shape[:2]

        cv2.putText(
            frame,
            "P3020 VISION // LIVE",
            (18, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            _LASER_GREEN,
            1,
            cv2.LINE_AA,
        )

        if detection is None:
            scan_y = int((0.5 + 0.5 * math.sin(now * 2.4)) * max(1, height - 1))
            glow = frame.copy()
            cv2.line(glow, (0, scan_y), (width - 1, scan_y), _LASER_GREEN, 9)
            cv2.addWeighted(glow, 0.14, frame, 0.86, 0, frame)
            cv2.line(frame, (0, scan_y), (width - 1, scan_y), _LASER_GREEN, 1)
            BoxDetectorNode._draw_label(frame, 18, 52, "SCANNING FOR BOX")
            return frame

        half_w = max(2.0, detection["w"] / 2.0)
        half_h = max(2.0, detection["h"] / 2.0)
        x1 = int(np.clip(detection["cx"] - half_w, 0, width - 1))
        y1 = int(np.clip(detection["cy"] - half_h, 0, height - 1))
        x2 = int(np.clip(detection["cx"] + half_w, 0, width - 1))
        y2 = int(np.clip(detection["cy"] + half_h, 0, height - 1))
        if x2 <= x1 or y2 <= y1:
            return frame

        corner = max(14, min(54, int(min(x2 - x1, y2 - y1) * 0.28)))
        segments = [
            ((x1, y1), (x1 + corner, y1)),
            ((x1, y1), (x1, y1 + corner)),
            ((x2, y1), (x2 - corner, y1)),
            ((x2, y1), (x2, y1 + corner)),
            ((x1, y2), (x1 + corner, y2)),
            ((x1, y2), (x1, y2 - corner)),
            ((x2, y2), (x2 - corner, y2)),
            ((x2, y2), (x2, y2 - corner)),
        ]

        glow = frame.copy()
        for start, end in segments:
            cv2.line(glow, start, end, _LASER_GREEN, 13, cv2.LINE_AA)
        cv2.addWeighted(glow, 0.17, frame, 0.83, 0, frame)
        for start, end in segments:
            cv2.line(frame, start, end, _LASER_GREEN, 2, cv2.LINE_AA)

        cx = int(np.clip(detection["cx"], 0, width - 1))
        cy = int(np.clip(detection["cy"], 0, height - 1))
        pulse = 8 + int(3 * (0.5 + 0.5 * math.sin(now * 7.0)))
        cv2.circle(frame, (cx, cy), pulse, _LASER_GREEN, 1, cv2.LINE_AA)
        cv2.line(frame, (cx - 18, cy), (cx + 18, cy), _LASER_GREEN, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - 18), (cx, cy + 18), _LASER_GREEN, 1, cv2.LINE_AA)

        scan_y = y1 + int((0.5 + 0.5 * math.sin(now * 3.5)) * (y2 - y1))
        scan_glow = frame.copy()
        cv2.line(scan_glow, (x1, scan_y), (x2, scan_y), _LASER_GREEN, 7)
        cv2.addWeighted(scan_glow, 0.13, frame, 0.87, 0, frame)
        cv2.line(frame, (x1, scan_y), (x2, scan_y), _LASER_GREEN, 1)

        label_y = max(52, y1 - 12)
        BoxDetectorNode._draw_label(
            frame,
            x1,
            label_y,
            f"TARGET LOCKED  {detection['conf'] * 100:.1f}%",
        )
        return frame

    @staticmethod
    def _draw_label(frame, x, baseline_y, text):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.48
        thickness = 1
        (text_w, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
        height, width = frame.shape[:2]
        x = int(np.clip(x, 0, max(0, width - text_w - 12)))
        baseline_y = int(np.clip(baseline_y, text_h + 10, height - 3))
        cv2.rectangle(
            frame,
            (x, baseline_y - text_h - 8),
            (x + text_w + 10, baseline_y + 4),
            (4, 20, 12),
            -1,
        )
        cv2.rectangle(
            frame,
            (x, baseline_y - text_h - 8),
            (x + text_w + 10, baseline_y + 4),
            _LASER_GREEN_DIM,
            1,
        )
        cv2.putText(
            frame,
            text,
            (x + 5, baseline_y),
            font,
            font_scale,
            _LASER_GREEN,
            thickness,
            cv2.LINE_AA,
        )

    def _publish_annotated(self, source_msg, annotated_rgb):
        output = Image()
        output.header = source_msg.header
        output.height, output.width = annotated_rgb.shape[:2]
        output.encoding = "rgb8"
        output.is_bigendian = 0
        output.step = output.width * 3
        output.data = annotated_rgb.tobytes()
        self.annotated_pub.publish(output)

    def _update_web_stream(self, annotated_rgb):
        bgr = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
        success, encoded = cv2.imencode(
            ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not success:
            return
        with self._frame_condition:
            self._latest_jpeg = encoded.tobytes()
            self._frame_sequence += 1
            self._frame_condition.notify_all()

    def destroy_node(self):
        try:
            self._inference_queue.put_nowait(None)
        except queue.Full:
            try:
                self._inference_queue.get_nowait()
            except queue.Empty:
                pass
            self._inference_queue.put_nowait(None)
        if self._inference_thread.is_alive():
            self._inference_thread.join(timeout=2.0)
        if self._stream_server is not None:
            self._stream_server.shutdown()
            self._stream_server.server_close()
        with self._frame_condition:
            self._frame_condition.notify_all()
        if self._stream_thread is not None and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetectorNode()
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
