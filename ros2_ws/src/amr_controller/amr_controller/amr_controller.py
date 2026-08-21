"""Forklift AMR 기본 동작을 명령하는 ROS 2 노드."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


# 각 단계에서 사용할 속도와 실행 시간을 순서대로 정의한다.
# linear: 전진·후진 속도(m/s)
# angular: 회전 속도(rad/s), 양수는 좌회전
# fork_up: True는 상승, False는 하강, None은 현재 상태 유지
TEST_STEPS = [
    {
        "name": "1. 직진",
        "duration": 3.0,
        "linear": 0.75,
        "angular": 0.0,
        "fork_up": False,
    },
    {
        "name": "2. 좌회전",
        "duration": 2.0,
        "linear": 0.75,
        "angular": 0.30,
        "fork_up": None,
    },
    {
        "name": "3. 목표 위치까지 직진",
        "duration": 3.0,
        "linear": 0.75,
        "angular": 0.0,
        "fork_up": None,
    },
    {
        "name": "4. 목표 위치 정지",
        "duration": 1.0,
        "linear": 0.0,
        "angular": 0.0,
        "fork_up": None,
    },
    {
        "name": "5. Fork 상승",
        "duration": 3.0,
        "linear": 0.0,
        "angular": 0.0,
        "fork_up": True,
    },
    {
        "name": "6. Fork 상승 상태 유지",
        "duration": 2.0,
        "linear": 0.0,
        "angular": 0.0,
        "fork_up": None,
    },
    {
        "name": "7. Fork 하강",
        "duration": 3.0,
        "linear": 0.0,
        "angular": 0.0,
        "fork_up": False,
    },
    {
        "name": "8. 후진",
        "duration": 3.0,
        "linear": -0.75,
        "angular": 0.0,
        "fork_up": None,
    },
    {
        "name": "9. 후진하면서 회전",
        "duration": 2.0,
        "linear": -0.75,
        "angular": -0.30,
        "fork_up": None,
    },
    {
        "name": "10. 시작 위치까지 후진",
        "duration": 3.0,
        "linear": -0.75,
        "angular": 0.0,
        "fork_up": None,
    },
]


class AMRController(Node):
    """기본 테스트 순서대로 Forklift에 명령을 보낸다."""

    def __init__(self):
        super().__init__("amr_controller")

        # namespace가 amr_a이면 실제 토픽은
        # /amr_a/cmd_vel과 /amr_a/fork_up이 된다.
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "cmd_vel",
            10,
        )

        self.fork_publisher = self.create_publisher(
            Bool,
            "fork_up",
            10,
        )

        self.current_step_index = 0
        self.step_started_at = self.get_clock().now()
        self.test_finished = False

        # 0.1초마다 현재 단계의 명령을 보낸다.
        self.timer = self.create_timer(
            0.1,
            self.update_test,
        )

        self.get_logger().info(
            "Forklift AMR 기본 동작 테스트 시작"
        )
        self.start_current_step()

    def start_current_step(self):
        """새 단계의 이름과 Fork 명령을 한 번 처리한다."""

        current_step = TEST_STEPS[
            self.current_step_index
        ]

        self.get_logger().info(
            current_step["name"]
        )

        fork_up = current_step["fork_up"]

        if fork_up is not None:
            fork_message = Bool()
            fork_message.data = fork_up
            self.fork_publisher.publish(
                fork_message
            )

    def update_test(self):
        """현재 단계의 주행 명령을 반복해서 전송한다."""

        if self.test_finished:
            self.publish_stop()
            return

        current_step = TEST_STEPS[
            self.current_step_index
        ]

        self.publish_cmd_vel(
            linear_velocity=current_step["linear"],
            angular_velocity=current_step["angular"],
        )

        now = self.get_clock().now()
        elapsed_time = (
            now - self.step_started_at
        ).nanoseconds / 1_000_000_000

        if elapsed_time < current_step["duration"]:
            return

        self.current_step_index += 1

        if self.current_step_index >= len(TEST_STEPS):
            self.finish_test()
            return

        self.step_started_at = now
        self.start_current_step()

    def publish_cmd_vel(
        self,
        linear_velocity,
        angular_velocity,
    ):
        """전진 속도와 회전 속도를 Twist로 전송한다."""

        command = Twist()
        command.linear.x = float(linear_velocity)
        command.angular.z = float(angular_velocity)

        self.cmd_vel_publisher.publish(command)

    def publish_stop(self):
        """정지 명령을 전송한다."""

        self.publish_cmd_vel(
            linear_velocity=0.0,
            angular_velocity=0.0,
        )

    def finish_test(self):
        """테스트를 종료하고 Forklift를 정지시킨다."""

        self.test_finished = True
        self.publish_stop()

        self.get_logger().info(
            "11. 시작 위치 복귀 완료"
        )


def main(args=None):
    """ROS 2 노드를 실행한다."""

    rclpy.init(args=args)
    node = AMRController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
