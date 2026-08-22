"""ROS 2 box detector for the P3020 pick&place exercise.

m0609_color_detector.py 와 같은 구조 (같은 PC 안에서 실행해도, 실제로는
"카메라를 가진 PC"와 "인식하는 PC"가 분리돼 있다는 걸 보여주는 것과 같은
구조): 카메라 이미지를 토픽으로 받아서, 딥러닝 모델로 박스를 찾고, 그
결과(픽셀 좌표)만 다시 토픽으로 돌려준다. 3D 위치 계산(깊이 역투영)은
카메라/깊이 정보를 직접 가진 시뮬레이션 쪽에서 한다 (실제 로봇에서도
카메라 depth는 로봇 쪽에만 있는 경우가 많은 것과 같은 이유).

Subscribes:
    /rgb        (sensor_msgs/msg/Image)

Publishes:
    /box_pixel  (geometry_msgs/msg/Point)
        x, y: 감지된 박스 중심의 픽셀 좌표
        z   : confidence (0~1). 감지 실패 시 이 토픽 자체를 발행하지 않는다.

이 노드는 Isaac Sim 프로세스가 아니라 일반 시스템 python3(+ /opt/ros/jazzy)
에서 실행한다:

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=55 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    python3 box_detector_node.py

onnxruntime 은 시스템 python3 에 --user 로 설치돼 있다 (apt의 numpy<2, cv2
와 호환되도록 numpy 는 그대로 두고 onnxruntime만 추가 설치함).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from object_detector import ObjectDetector

MODEL_PATH = "/home/rokey/Downloads/parcel_box_yolo_model/best.onnx"
IMAGE_TOPIC = "/rgb"
BOX_PIXEL_TOPIC = "/box_pixel"

_ENCODING_CHANNELS = {"rgb8": 3, "bgr8": 3}


class BoxDetectorNode(Node):
    def __init__(self):
        super().__init__("p3020_box_detector")

        self.declare_parameter("image_topic", IMAGE_TOPIC)
        self.declare_parameter("box_pixel_topic", BOX_PIXEL_TOPIC)
        self.declare_parameter("model_path", MODEL_PATH)
        self.declare_parameter("conf_threshold", 0.5)

        image_topic = str(self.get_parameter("image_topic").value)
        pixel_topic = str(self.get_parameter("box_pixel_topic").value)
        model_path = str(self.get_parameter("model_path").value)
        conf_threshold = float(self.get_parameter("conf_threshold").value)

        self.detector = ObjectDetector(model_path, conf_threshold=conf_threshold)

        # 카메라 퍼블리셔는 보통 sensor-data QoS(best effort)를 쓴다.
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.pixel_pub = self.create_publisher(Point, pixel_topic, 10)

        self.get_logger().info(f"listening: {image_topic} | publishing: {pixel_topic}")

    @staticmethod
    def imgmsg_to_rgb(msg: Image):
        channels = _ENCODING_CHANNELS.get(msg.encoding)
        if channels is None:
            return None
        expected = msg.height * msg.width * channels
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size < expected:
            return None
        arr = arr[:expected].reshape((msg.height, msg.width, channels))
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return arr

    def image_callback(self, msg: Image):
        rgb = self.imgmsg_to_rgb(msg)
        if rgb is None:
            self.get_logger().warn(
                f'unsupported or truncated image (encoding="{msg.encoding}")',
                throttle_duration_sec=5.0,
            )
            return

        det = self.detector.detect(rgb)
        if det is None:
            return

        point = Point()
        point.x = det["cx"]
        point.y = det["cy"]
        point.z = det["conf"]
        self.pixel_pub.publish(point)
        self.get_logger().info(
            f"box detected  pixel=({det['cx']:.1f},{det['cy']:.1f})  conf={det['conf']:.3f}"
        )


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
