"""locate_box_node.py(vision_node 패키지)의 /locate_box 서비스를
p3020_pick_place_poc.py 가 쓸 수 있는 표준 토픽으로 중계한다.

왜 필요한가
-----------
Isaac Sim(이 파일이 아니라 p3020_pick_place_poc.py 쪽)은 자체 내장 Python
(Kit 번들, 3.11)과 자체 내장 rclpy(jazzy 브릿지, 역시 3.11용으로 빌드됨)를
쓴다. 반면 logistics_interfaces 패키지는 시스템 ROS2(/opt/ros/jazzy,
Python 3.12)로 colcon build 했기 때문에, 컴파일된 타입 바인딩이
cpython-312 전용이라 Isaac Sim 프로세스 안에서
`from logistics_interfaces.srv import LocateBox` 를 하면 ImportError가 난다
(직접 재현 확인함). NVIDIA가 배포하는 IsaacSim-ros_workspaces의
custom_message 예제도 Isaac 내장 Python이 아니라 별도 Ubuntu/Python 3.12
Docker 환경으로 빌드하는 걸 보면, 커스텀 ROS2 메시지/서비스 타입을 Isaac
Sim 프로세스 안에 직접 import하는 공식 경로는 없는 것으로 보인다.

반면 sensor_msgs, geometry_msgs 같은 표준 패키지는 Isaac Sim이 자체
Python 3.11용으로 이미 빌드해서 들고 있어서 문제없이 쓸 수 있다
(p3020_pick_place_poc.py가 /rgb, /depth 를 Image로 발행하는 것과 동일).

그래서 이 노드는 시스템 Python(logistics_interfaces와 같은 빌드)에서
돌면서 /locate_box 서비스를 대신 호출하고, 결과를 표준 geometry_msgs/Point
토픽으로 다시 뿌려준다. p3020_pick_place_poc.py는 이 토픽만 구독하면 되므로
커스텀 타입을 전혀 import할 필요가 없다.

Subscribes:
    (없음 -- 내부 타이머로 주기적으로 /locate_box 를 호출한다)

Publishes:
    box_position_camera_topic (기본 /box_position_camera, geometry_msgs/Point)
        locate_box_node.py가 반환한 카메라 좌표계(광학 프레임: X=오른쪽,
        Y=아래, Z=전방) 3D 좌표. 박스가 검출되지 않으면 이 토픽 자체를
        발행하지 않는다 (box_detector_node.py의 /box_pixel 관례와 동일).

release_lock=true로 매번 호출하는 이유: p3020_pick_place_poc.py는 사이클마다
"새 박스"이므로 항상 최신 검출값이 필요하다 (locate_box_pipeline.md 흐름
문서 참고). 이 릴레이가 그 정책을 대신 적용해서, poc 쪽은 그냥 최신 토픽
값만 받아쓰면 된다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from logistics_interfaces.srv import LocateBox


class LocateBoxRelay(Node):
    def __init__(self):
        super().__init__("locate_box_relay")

        self.declare_parameter("locate_box_service", "locate_box")
        self.declare_parameter("box_position_camera_topic", "/box_position_camera")
        self.declare_parameter("poll_period_sec", 0.1)

        service_name = str(self.get_parameter("locate_box_service").value)
        topic_name = str(self.get_parameter("box_position_camera_topic").value)
        poll_period = float(self.get_parameter("poll_period_sec").value)

        self._client = self.create_client(LocateBox, service_name)
        self._pub = self.create_publisher(Point, topic_name, 10)
        self._call_in_flight = False
        self._last_published = None  # (x, y, z) | None

        self._timer = self.create_timer(poll_period, self._poll)
        self.get_logger().info(
            f"locate_box_relay ready: {service_name} (service) -> {topic_name} (topic)"
        )

    def _poll(self):
        if self._call_in_flight or not self._client.service_is_ready():
            return
        self._call_in_flight = True
        request = LocateBox.Request()
        request.release_lock = True
        future = self._client.call_async(request)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self._call_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"/locate_box call failed: {exc}", throttle_duration_sec=5.0)
            return
        if not response.box_detected:
            return
        p = response.box_position
        current = (p.x, p.y, p.z)
        # locate_box_node은 새로 검출되지 않아도(카메라가 한동안 조용해도)
        # "마지막으로 검출된 값"을 계속 즉시 돌려준다 (release_lock=true를
        # 매번 보내도 그렇다 -- 서버 쪽 설계가 그럼). 그래서 여기서 매 poll을
        # 그대로 토픽에 흘려보내면, poc가 "첫 응답 오면 바로 멈추는" 방식이라
        # 실제로는 완전히 낡은(이전 박스의) 값을 즉시 받아버리는 문제가 있었다
        # (p3020_pick_place_poc.py 통합 중 재현/확인함). 그래서 "값이 실제로
        # 바뀌었을 때만" 발행해서, 진짜 새 검출 이벤트만 토픽에 흘러가게 한다.
        if current == self._last_published:
            return
        self._last_published = current
        self._pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = LocateBoxRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
