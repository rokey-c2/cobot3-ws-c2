"""ROS 2 color detector for the M0609 Isaac Sim exercise.

Subscribes:
    /rgb       (sensor_msgs/msg/Image)

Publishes:
    /color_id  (std_msgs/msg/Int32)
        0: no reliable detection
        1: blue cube
        2: green cube

The image topic, result topic, ROI margin, minimum contour area, and debug
window can all be changed with ROS parameters.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


UNKNOWN = 0
BLUE = 1
GREEN = 2

# cv_bridge's C extension is built against NumPy 1.x and segfaults on this
# machine's NumPy 2.x when it actually converts an image, so raw
# sensor_msgs/Image buffers are decoded manually instead.
_ENCODING_CHANNELS = {"rgb8": 3, "bgr8": 3, "mono8": 1}

# OpenCV HSV ranges: H=0..179, S=0..255, V=0..255.
# Blue sits roughly in H=100..130 for typical saturated cube materials.
BLUE_LOWER = np.array([100, 80, 50], dtype=np.uint8)
BLUE_UPPER = np.array([130, 255, 255], dtype=np.uint8)
GREEN_LOWER = np.array([35, 70, 50], dtype=np.uint8)
GREEN_UPPER = np.array([85, 255, 255], dtype=np.uint8)


class ColorDetector(Node):
    """Detect a blue or green object in the centre of a ROS image."""

    def __init__(self) -> None:
        super().__init__("m0609_color_detector")

        self.declare_parameter("image_topic", "/rgb")
        self.declare_parameter("color_topic", "/color_id")
        self.declare_parameter("min_area", 500.0)
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("roi_margin", 0.20)
        self.declare_parameter("debug_view", False)

        image_topic = str(self.get_parameter("image_topic").value)
        color_topic = str(self.get_parameter("color_topic").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.stable_frames = max(
            1, int(self.get_parameter("stable_frames").value)
        )
        self.roi_margin = float(self.get_parameter("roi_margin").value)
        self.debug_view = bool(self.get_parameter("debug_view").value)

        if not 0.0 <= self.roi_margin < 0.5:
            raise ValueError("roi_margin must be at least 0.0 and below 0.5")

        self.candidate_id = UNKNOWN
        self.candidate_count = 0
        self.stable_color_id = UNKNOWN
        self.last_logged_id = None

        # Camera publishers commonly use sensor-data QoS (best effort).
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.color_pub = self.create_publisher(Int32, color_topic, 10)

        # Publish repeatedly so Isaac Sim can recover after a late start or a
        # temporary network interruption.
        self.publish_timer = self.create_timer(0.2, self.publish_color)

        self.get_logger().info(
            f"listening: {image_topic} | publishing: {color_topic}"
        )

    @staticmethod
    def imgmsg_to_bgr(msg: Image) -> np.ndarray | None:
        """Decode a sensor_msgs/Image into an OpenCV BGR array without cv_bridge."""
        channels = _ENCODING_CHANNELS.get(msg.encoding)
        if channels is None:
            return None

        expected = msg.height * msg.width * channels
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size < expected:
            return None

        arr = arr[:expected].reshape((msg.height, msg.width, channels))
        if msg.encoding == "rgb8":
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if msg.encoding == "mono8":
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        return arr

    @staticmethod
    def clean_mask(mask: np.ndarray) -> np.ndarray:
        """Remove isolated pixels and fill small holes."""
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    @staticmethod
    def largest_contour_area(mask: np.ndarray) -> float:
        """Return the area of the largest connected region in a mask."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return 0.0
        return float(max(cv2.contourArea(contour) for contour in contours))

    def detect_color(
        self, frame: np.ndarray
    ) -> tuple[int, float, float, np.ndarray, np.ndarray]:
        """Return detected ID, blue/green areas, ROI image, and chosen mask."""
        height, width = frame.shape[:2]
        x_margin = int(width * self.roi_margin)
        y_margin = int(height * self.roi_margin)
        roi = frame[
            y_margin : height - y_margin,
            x_margin : width - x_margin,
        ]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        blue_mask = self.clean_mask(
            cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
        )
        green_mask = self.clean_mask(
            cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        )

        blue_area = self.largest_contour_area(blue_mask)
        green_area = self.largest_contour_area(green_mask)

        if max(blue_area, green_area) < self.min_area:
            return UNKNOWN, blue_area, green_area, roi, np.zeros_like(blue_mask)
        if blue_area > green_area:
            return BLUE, blue_area, green_area, roi, blue_mask
        return GREEN, blue_area, green_area, roi, green_mask

    def update_stable_result(self, detected_id: int) -> None:
        """Accept a result only after it repeats for several frames."""
        if detected_id == self.candidate_id:
            self.candidate_count += 1
        else:
            self.candidate_id = detected_id
            self.candidate_count = 1

        if self.candidate_count >= self.stable_frames:
            self.stable_color_id = detected_id

    def image_callback(self, msg: Image) -> None:
        """Convert a ROS image, detect its colour, and update the result."""
        frame = self.imgmsg_to_bgr(msg)
        if frame is None:
            self.get_logger().warn(
                f'unsupported or truncated image (encoding="{msg.encoding}")',
                throttle_duration_sec=5.0,
            )
            return

        detected_id, blue_area, green_area, roi, mask = self.detect_color(frame)
        self.update_stable_result(detected_id)

        if self.debug_view:
            labels = {UNKNOWN: "UNKNOWN", BLUE: "BLUE", GREEN: "GREEN"}
            debug = roi.copy()
            cv2.putText(
                debug,
                (
                    f"raw={labels[detected_id]} "
                    f"stable={labels[self.stable_color_id]} "
                    f"B={blue_area:.0f} G={green_area:.0f}"
                ),
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("m0609 color detector", debug)
            cv2.imshow("m0609 selected mask", mask)
            cv2.waitKey(1)

    def publish_color(self) -> None:
        """Publish the current stable result at 5 Hz."""
        message = Int32()
        message.data = int(self.stable_color_id)
        self.color_pub.publish(message)

        if self.last_logged_id != self.stable_color_id:
            names = {UNKNOWN: "UNKNOWN", BLUE: "BLUE", GREEN: "GREEN"}
            self.get_logger().info(
                f"color_id={self.stable_color_id} "
                f"({names[self.stable_color_id]})"
            )
            self.last_logged_id = self.stable_color_id

    def close_windows(self) -> None:
        if self.debug_view:
            cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_windows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()