"""Rate-limited ROS 2 Image -> lightweight MJPEG stream.

Only the newest frames are encoded. Encoding is capped by ``max_fps`` and large
sources can be downscaled before JPEG compression to avoid wasting CPU/network
bandwidth on browser monitoring.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

IMAGE_TOPIC = "/top_view/rgb"
STREAM_HOST = "0.0.0.0"
STREAM_PORT = 8092
STREAM_PATH = "/stream.mjpg"

_ENCODING_CHANNELS = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
}

_IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class _MjpegServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _MjpegHandler(BaseHTTPRequestHandler):
    server_version = "ControlTowerStream/2.0"

    def do_GET(self):
        source = self.server.frame_source
        request_path = self.path.split("?", 1)[0]

        if request_path.rstrip("/") == "/health":
            payload = json.dumps(
                {
                    "status": "ok",
                    "image_topic": source.image_topic,
                    "stream": source.stream_path,
                    "frame_ready": source.has_stream_frame,
                    "max_fps": source.max_fps,
                    "max_width": source.max_width,
                }
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(payload)
            return

        if request_path != source.stream_path:
            self.send_error(404, "stream not found")
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
            while rclpy.ok():
                jpeg, sequence = source.wait_for_stream_frame(last_sequence, timeout=2.0)
                if jpeg is None or sequence == last_sequence:
                    continue
                last_sequence = sequence
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def log_message(self, _format, *_args):
        return


class TopViewStreamNode(Node):
    def __init__(self):
        super().__init__("control_tower_image_stream")

        self.declare_parameter("image_topic", IMAGE_TOPIC)
        self.declare_parameter("stream_host", STREAM_HOST)
        self.declare_parameter("stream_port", STREAM_PORT)
        self.declare_parameter("stream_path", STREAM_PATH)
        self.declare_parameter("jpeg_quality", 68)
        self.declare_parameter("max_fps", 8.0)
        self.declare_parameter("max_width", 960)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.stream_host = str(self.get_parameter("stream_host").value)
        self.stream_port = int(self.get_parameter("stream_port").value)
        self.stream_path = self._normalise_stream_path(
            str(self.get_parameter("stream_path").value)
        )
        self.jpeg_quality = int(np.clip(self.get_parameter("jpeg_quality").value, 40, 95))
        self.max_fps = max(0.5, float(self.get_parameter("max_fps").value))
        self.max_width = max(0, int(self.get_parameter("max_width").value))
        self._encode_interval = 1.0 / self.max_fps
        self._last_encode_at = 0.0

        self._frame_condition = threading.Condition()
        self._latest_jpeg = None
        self._frame_sequence = 0
        self._stream_server = None
        self._stream_thread = None

        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            _IMAGE_QOS,
        )
        self._start_stream_server()

        self.get_logger().info(
            f"stream: {self.image_topic} -> "
            f"http://{self.stream_host}:{self.stream_port}{self.stream_path} | "
            f"<= {self.max_fps:.1f} Hz | "
            f"max_width={self.max_width or 'source'} | jpeg={self.jpeg_quality}"
        )

    @staticmethod
    def _normalise_stream_path(path):
        path = path.strip() or STREAM_PATH
        return path if path.startswith("/") else f"/{path}"

    @property
    def has_stream_frame(self):
        with self._frame_condition:
            return self._latest_jpeg is not None

    def wait_for_stream_frame(self, last_sequence, timeout):
        with self._frame_condition:
            if self._frame_sequence == last_sequence:
                self._frame_condition.wait(timeout=timeout)
            return self._latest_jpeg, self._frame_sequence

    def _start_stream_server(self):
        try:
            self._stream_server = _MjpegServer(
                (self.stream_host, self.stream_port),
                _MjpegHandler,
            )
            self._stream_server.frame_source = self
            self._stream_thread = threading.Thread(
                target=self._stream_server.serve_forever,
                name="control-tower-mjpeg",
                daemon=True,
            )
            self._stream_thread.start()
        except OSError as error:
            self._stream_server = None
            self.get_logger().error(
                f"MJPEG server could not bind {self.stream_host}:{self.stream_port}: {error}"
            )

    def _image_callback(self, message: Image):
        now = time.monotonic()
        if now - self._last_encode_at < self._encode_interval:
            return
        self._last_encode_at = now

        bgr = self._image_to_bgr(message)
        if bgr is None:
            self.get_logger().warning(
                f'unsupported/truncated image encoding="{message.encoding}"',
                throttle_duration_sec=5.0,
            )
            return

        if self.max_width > 0 and bgr.shape[1] > self.max_width:
            scale = self.max_width / bgr.shape[1]
            target_height = max(1, int(round(bgr.shape[0] * scale)))
            bgr = cv2.resize(
                bgr,
                (self.max_width, target_height),
                interpolation=cv2.INTER_AREA,
            )

        success, encoded = cv2.imencode(
            ".jpg",
            bgr,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not success:
            return

        with self._frame_condition:
            self._latest_jpeg = encoded.tobytes()
            self._frame_sequence += 1
            self._frame_condition.notify_all()

    @staticmethod
    def _image_to_bgr(message: Image):
        channels = _ENCODING_CHANNELS.get(message.encoding)
        if channels is None:
            return None

        row_size = message.width * channels
        if message.step < row_size:
            return None

        array = np.frombuffer(message.data, dtype=np.uint8)
        required = message.height * message.step
        if array.size < required:
            return None

        rows = array[:required].reshape((message.height, message.step))
        image = rows[:, :row_size].reshape((message.height, message.width, channels))

        if message.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if message.encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if message.encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return np.ascontiguousarray(image)

    def destroy_node(self):
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
    node = TopViewStreamNode()
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
