"""LocateBox / ConfirmGrasp 서비스 노드.

역할: rgb/depth 이미지 토픽을 구독해 YOLO(1클래스: box)로 박스를 검출하고,
depth 값을 픽셀 좌표와 결합(pinhole 역투영)해 카메라 좌표계 3D 좌표를 계산한다.

LocateBox: WAITING/DETECTED/LOCKED 상태를 관리한다.
    release_lock=false : 잠겨 있으면 잠긴 좌표 유지, 아니면 새로 검출 후 잠금.
    release_lock=true  : 잠금 해제 후 새로 검출(해서 다시 잠금).
    (자세한 설계 이유는 logistics_interfaces/srv/LocateBox.srv 주석 참고)

ConfirmGrasp: 로봇팔이 보낸 rgb 이미지로 박스가 더 이상 검출되지 않으면
    흡착 성공으로 간주한다. 이 판단 기준은 아직 팀 확정 전 placeholder.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

from logistics_interfaces.srv import LocateBox, ConfirmGrasp

from vision_node.object_detector import ObjectDetector


class LocateBoxNode(Node):
    def __init__(self):
        super().__init__('locate_box_node')

        self.declare_parameter('rgb_topic', '/rgb')
        self.declare_parameter('depth_topic', '/depth')
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        # 640x480, 수평 FOV 90.53도는 Isaac Sim 카메라 prim
        # (P3020_mount_vgp20_rsd455_1/World1.usd,
        # /World/vgp20/rsd455/RSD455/Camera_Pseudo_Depth)의 실측
        # focalLength=1.93 / horizontalAperture=3.896 로부터 계산한 값
        # (config/vision.yaml 주석 참고). bag 재생(실카메라 없음) 등 다른
        # 카메라를 쓸 때는 파라미터로 반드시 덮어써야 한다.
        self.declare_parameter('horizontal_fov_deg', 90.53)
        self.declare_parameter('sync_slop_sec', 0.05)
        # _latest_detection이 이 시간(초)보다 오래되면 "박스가 없어졌을 수도
        # 있다"고 보고 box_detected=False로 취급한다. 원래는 "한 번이라도
        # 검출된 적 있으면" 그 값을 시간 제한 없이 계속 반환했는데, 그러면
        # 호출자가 release_lock=true로 불러도 실제로는 몇 초~몇십 초 전의
        # (박스가 이미 다른 곳으로 옮겨졌을 수 있는) 낡은 좌표를 즉시 돌려받게
        # 된다 -- p3020_pick_place_poc.py 통합 중 실제로 재현/확인한 버그.
        self.declare_parameter('detection_timeout_sec', 1.0)

        model_path = self.get_parameter('model_path').value
        if not model_path:
            raise RuntimeError(
                "'model_path' 파라미터가 비어 있음. YOLO onnx 모델 경로를 지정해야 한다."
            )

        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        hfov_deg = self.get_parameter('horizontal_fov_deg').value

        # 카메라 intrinsics placeholder: 정사각 픽셀 가정, 수평 FOV로 fx 계산.
        # (bag에 /camera_info 없어서 CLAUDE.md 합의대로 임시 계산)
        self._fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self._fy = self._fx
        self._cx = width / 2.0
        self._cy = height / 2.0

        self._bridge = CvBridge()
        self._detector = ObjectDetector(
            model_path, conf_threshold=self.get_parameter('conf_threshold').value
        )

        self._latest_detection = None  # geometry_msgs/Point | None
        self._latest_detection_time = None  # rclpy.time.Time | None
        self._locked = False
        self._locked_position = None  # geometry_msgs/Point | None

        # 카메라 토픽 관례대로 best-effort QoS 사용 (레코딩된 bag도 best_effort로 기록됨)
        rgb_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('rgb_topic').value, qos_profile=qos_profile_sensor_data
        )
        depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('depth_topic').value, qos_profile=qos_profile_sensor_data
        )
        # queue_size가 크면(원래 10) 발행이 한동안 끊겼다가 재개될 때(로봇팔이
        # 스캔 자세로 이동하는 동안 등) 큐에 남아있던 오래된 메시지가 새
        # 메시지와 잘못 매칭돼 낡은 검출을 반환하는 사례를 발견함(직접 재현).
        # 작게 유지해서 그런 오래된 잔여 메시지가 오래 안 남게 한다.
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=2,
            slop=self.get_parameter('sync_slop_sec').value,
        )
        self._sync.registerCallback(self._on_synced_frame)

        self.create_service(LocateBox, 'locate_box', self._handle_locate_box)
        self.create_service(ConfirmGrasp, 'confirm_grasp', self._handle_confirm_grasp)

        self.get_logger().info('locate_box_node ready (locate_box, confirm_grasp)')

    def _on_synced_frame(self, rgb_msg: Image, depth_msg: Image):
        # 가정: rgb_topic은 rgb 채널 순서(encoding rgb8), depth_topic은 미터 단위
        # distance_to_image_plane(encoding 32FC1). 실카메라 연동 시 재확인 필요.
        rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        detection = self._detector.detect(rgb)
        self._latest_detection = self._to_point(detection, depth)
        self._latest_detection_time = self.get_clock().now()

    def _detection_is_fresh(self) -> bool:
        if self._latest_detection is None or self._latest_detection_time is None:
            return False
        age_sec = (self.get_clock().now() - self._latest_detection_time).nanoseconds / 1e9
        timeout = self.get_parameter('detection_timeout_sec').value
        return age_sec <= timeout

    def _to_point(self, detection, depth):
        if detection is None:
            return None

        h, w = depth.shape[:2]
        px = int(round(min(max(detection['cx'], 0), w - 1)))
        py = int(round(min(max(detection['cy'], 0), h - 1)))
        z = float(depth[py, px])
        if not math.isfinite(z) or z <= 0.0:
            return None

        x = (detection['cx'] - self._cx) * z / self._fx
        y = (detection['cy'] - self._cy) * z / self._fy
        return Point(x=x, y=y, z=z)

    def _handle_locate_box(self, request: LocateBox.Request, response: LocateBox.Response):
        if request.release_lock:
            self._locked = False
            self._locked_position = None

        if self._locked and self._locked_position is not None:
            response.box_detected = True
            response.box_position = self._locked_position
            response.position_locked = True
            return response

        if self._detection_is_fresh():
            self._locked = True
            self._locked_position = self._latest_detection
            response.box_detected = True
            response.box_position = self._latest_detection
            response.position_locked = False
        else:
            response.box_detected = False
            response.box_position = Point(x=0.0, y=0.0, z=0.0)
            response.position_locked = False

        return response

    def _handle_confirm_grasp(self, request: ConfirmGrasp.Request, response: ConfirmGrasp.Response):
        rgb = self._bridge.imgmsg_to_cv2(request.rgb_image, desired_encoding='passthrough')
        detection = self._detector.detect(rgb)

        # Placeholder 판단 기준: 흡착 성공 시 그리퍼가 박스를 원래 위치에서
        # 들어올려 카메라 시야에서 더 이상 검출되지 않는다고 가정한다.
        # depth_image는 아직 사용하지 않음 - 실제 판단 기준은 팀과 확정 필요.
        response.grasp_success = detection is None
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocateBoxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
