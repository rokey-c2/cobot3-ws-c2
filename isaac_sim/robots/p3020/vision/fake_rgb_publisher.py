"""box_detector_node.py 를 Isaac Sim 없이 테스트하기 위한 가짜 카메라 퍼블리셔.

Isaac Sim 쪽 p3020_pick_place_poc.py 가 하는 일(카메라 RGBA 프레임을
sensor_msgs/Image, encoding=rgb8 로 /rgb 토픽에 발행) 만 흉내낸다.

동작
----
- --image 로 이미지 파일(png/jpg)을 주면 그 프레임을 반복 발행한다.
  (팀원 YOLO 모델이 실제로 박스를 검출하는지 보려면 택배상자 사진을 넣으면 됨)
- 안 주면 회색 배경에 초록 사각형을 그린 합성 프레임을 발행한다.
  (검출은 안 되지만 토픽 배관 / 다른 PC 에서 뷰어로 보이는지 확인용)

사용법
------
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=111 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

    # 합성 프레임
    python3 fake_rgb_publisher.py

    # 실제 상자 사진으로
    python3 fake_rgb_publisher.py --image /path/to/box.jpg --rate 10

cv_bridge 는 이 환경에서 안 쓴다(box_detector_node.py 와 같은 이유). 메시지
바이트를 직접 채운다.
"""

import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def make_synthetic_frame(width: int, height: int) -> np.ndarray:
    """회색 배경 + 초록 사각형 + 안내 텍스트 (RGB, uint8)."""
    frame = np.full((height, width, 3), 90, dtype=np.uint8)
    x1, y1 = width // 3, height // 3
    x2, y2 = 2 * width // 3, 2 * height // 3
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), -1)
    cv2.putText(
        frame, "fake /rgb (no real box)", (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA,
    )
    return frame


def load_image_rgb(path: str) -> np.ndarray:
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"이미지를 못 읽음: {path}")
    return bgr[:, :, ::-1].copy()  # BGR -> RGB


def rgb_to_imgmsg(rgb: np.ndarray, stamp) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = "camera_rgb_optical_frame"
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
    return msg


class FakeRgbPublisher(Node):
    def __init__(self, args):
        super().__init__("fake_rgb_publisher")
        self._pub = self.create_publisher(Image, args.topic, qos_profile_sensor_data)

        if args.image:
            self._frame = load_image_rgb(args.image)
            src = args.image
        else:
            self._frame = make_synthetic_frame(args.width, args.height)
            src = "synthetic"

        h, w = self._frame.shape[:2]
        period = 1.0 / max(args.rate, 0.1)
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"publishing {src} {w}x{h} rgb8 -> {args.topic} @ {args.rate:.1f} Hz"
        )

    def _tick(self):
        self._pub.publish(rgb_to_imgmsg(self._frame, self.get_clock().now().to_msg()))


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/rgb")
    parser.add_argument("--image", default=None, help="발행할 이미지 파일 (없으면 합성 프레임)")
    parser.add_argument("--rate", type=float, default=10.0, help="발행 Hz")
    parser.add_argument("--width", type=int, default=640, help="합성 프레임 가로")
    parser.add_argument("--height", type=int, default=480, help="합성 프레임 세로")
    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = FakeRgbPublisher(parsed)
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
