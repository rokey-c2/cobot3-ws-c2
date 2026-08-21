"""
ROS2와 ForkliftController를 연결한다.

구독 토픽:
    /amr_a/cmd_vel
    /amr_a/fork_up

메시지:
    geometry_msgs/msg/Twist
    std_msgs/msg/Bool
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


# 마지막 명령 이후 이 시간이 지나면 자동 정지
COMMAND_TIMEOUT_SECONDS = 0.5


class ForkliftRos2Adapter:
    """ROS2 cmd_vel 명령을 ForkliftController에 전달한다."""

    def __init__(
        self,
        forklift_controller,
        namespace="amr_a",
    ):
        self.forklift_controller = (
            forklift_controller
        )

        # 아직 ROS2가 초기화되지 않았다면 초기화
        if not rclpy.ok():
            rclpy.init(args=None)

        # namespace를 적용하면 실제 토픽은
        # /amr_a/cmd_vel이 된다.
        self.node = Node(
            node_name="forklift_ros2_adapter",
            namespace=namespace,
        )

        self.cmd_vel_subscription = (
            self.node.create_subscription(
                Twist,
                "cmd_vel",
                self.cmd_vel_callback,
                10,
            )
        )

        self.fork_subscription = (
            self.node.create_subscription(
                Bool,
                "fork_up",
                self.fork_command_callback,
                10,
            )
        )

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.has_received_command = False
        self.last_command_time = time.monotonic()

        print("[ROS2] Forklift ROS2 Adapter 시작")
        print("[ROS2] 구독 토픽: /amr_a/cmd_vel")
        print("[ROS2] 구독 토픽: /amr_a/fork_up")

    def cmd_vel_callback(self, message):
        """새로운 cmd_vel 명령을 저장한다."""

        self.linear_velocity = float(
            message.linear.x
        )
        self.angular_velocity = float(
            message.angular.z
        )

        self.has_received_command = True
        self.last_command_time = time.monotonic()

    def fork_command_callback(self, message):
        """Fork 상승 또는 하강 명령을 실행한다."""

        if message.data:
            print("[ROS2] Fork 상승 명령 수신")
            self.forklift_controller.lift_up()
            return

        print("[ROS2] Fork 하강 명령 수신")
        self.forklift_controller.lift_down()

    def update(self):
        """
        ROS2 메시지를 확인하고 Forklift를 제어한다.

        main.py의 시뮬레이션 반복문에서
        매 프레임 호출한다.
        """

        # 대기 중인 ROS2 메시지 확인
        rclpy.spin_once(
            self.node,
            timeout_sec=0.0,
        )

        # 아직 명령을 받은 적이 없다면 정지
        if not self.has_received_command:
            self.forklift_controller.stop()
            return

        current_time = time.monotonic()

        command_age = (
            current_time
            - self.last_command_time
        )

        # 일정 시간 동안 새 명령이 없으면 안전 정지
        if command_age > COMMAND_TIMEOUT_SECONDS:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        self.forklift_controller.drive_from_cmd_vel(
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
        )

    def shutdown(self):
        """ROS2 노드를 안전하게 종료한다."""

        self.forklift_controller.stop()

        self.node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("[ROS2] Forklift ROS2 Adapter 종료")
