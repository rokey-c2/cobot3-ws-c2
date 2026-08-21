"""Nav2, 수동 운전, 안전 정지를 Forklift 최종 속도로 중재한다."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from amr_controller.velocity_mux_policy import (
    MANUAL_SOURCE,
    NAVIGATION_SOURCE,
    select_velocity_source,
)


class VelocityMux(Node):
    """수동 명령에 우선권을 주고 정지 상태에서는 항상 zero Twist를 낸다."""

    def __init__(self):
        super().__init__("velocity_mux")

        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("navigation_enabled_on_start", False)

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        self.navigation_enabled = bool(
            self.get_parameter("navigation_enabled_on_start").value
        )

        if publish_rate <= 0.0 or self.command_timeout <= 0.0:
            raise ValueError("publish_rate and command_timeout must be positive")

        self.emergency_stop = False
        self.manual_command = Twist()
        self.navigation_command = Twist()
        self.last_manual_time = None
        self.last_navigation_time = None
        self.last_source = None

        self.publisher = self.create_publisher(Twist, "drive_cmd_vel", 10)
        self.create_subscription(
            Twist,
            "manual_cmd_vel",
            self.manual_callback,
            10,
        )
        self.create_subscription(
            Twist,
            "cmd_vel",
            self.navigation_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "navigation_enabled",
            self.navigation_enabled_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "emergency_stop",
            self.emergency_stop_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)
        self.get_logger().info(
            "velocity mux: emergency_stop > manual_cmd_vel > cmd_vel"
        )
        self.get_logger().info(
            f"navigation initially enabled: {self.navigation_enabled}"
        )

    def manual_callback(self, message):
        self.manual_command = message
        self.last_manual_time = time.monotonic()

    def navigation_callback(self, message):
        self.navigation_command = message
        self.last_navigation_time = time.monotonic()

    def navigation_enabled_callback(self, message):
        self.navigation_enabled = bool(message.data)

    def emergency_stop_callback(self, message):
        self.emergency_stop = bool(message.data)

    @staticmethod
    def command_age(last_time, now):
        if last_time is None:
            return None
        return now - last_time

    def publish_command(self):
        now = time.monotonic()
        source = select_velocity_source(
            emergency_stop=self.emergency_stop,
            navigation_enabled=self.navigation_enabled,
            manual_age=self.command_age(self.last_manual_time, now),
            navigation_age=self.command_age(self.last_navigation_time, now),
            command_timeout=self.command_timeout,
        )

        if source == MANUAL_SOURCE:
            command = self.manual_command
        elif source == NAVIGATION_SOURCE:
            command = self.navigation_command
        else:
            command = Twist()

        self.publisher.publish(command)

        if source != self.last_source:
            self.get_logger().info(f"active velocity source: {source}")
            self.last_source = source


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT 처리 후 rclpy context가 먼저 종료된 경우 publish하지 않는다.
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
