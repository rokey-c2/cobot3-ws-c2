"""한 대의 ForkliftB를 목표 XY 좌표까지 자동 주행시키는 ROS 2 노드."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from amr_controller.navigation_math import calculate_velocity_command


def quaternion_to_yaw(orientation):
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class GoalNavigator(Node):
    """Odometry 피드백을 사용해 goal_pose까지 폐루프 주행한다."""

    def __init__(self):
        super().__init__("goal_navigator")

        self.declare_parameter("control_period", 0.1)
        self.declare_parameter("linear_gain", 0.65)
        self.declare_parameter("angular_gain", 1.4)
        self.declare_parameter("min_linear_speed", 0.15)
        self.declare_parameter("max_linear_speed", 0.75)
        self.declare_parameter("max_angular_speed", 0.55)
        self.declare_parameter("distance_tolerance", 0.15)
        self.declare_parameter("odom_timeout", 0.5)

        self.control_period = float(
            self.get_parameter("control_period").value
        )
        self.linear_gain = float(
            self.get_parameter("linear_gain").value
        )
        self.angular_gain = float(
            self.get_parameter("angular_gain").value
        )
        self.min_linear_speed = float(
            self.get_parameter("min_linear_speed").value
        )
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.distance_tolerance = float(
            self.get_parameter("distance_tolerance").value
        )
        self.odom_timeout = float(
            self.get_parameter("odom_timeout").value
        )

        namespace = self.get_namespace().strip("/")
        self.odom_frame = f"{namespace}/odom" if namespace else "odom"

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "cmd_vel",
            10,
        )
        self.goal_reached_publisher = self.create_publisher(
            Bool,
            "goal_reached",
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self.goal_callback,
            10,
        )

        self.current_pose = None
        self.last_odom_time = None
        self.goal = None
        self.waiting_for_odom_logged = False

        self.timer = self.create_timer(
            self.control_period,
            self.control_step,
        )

        self.get_logger().info(
            "ForkliftB 단일 목표점 자율주행 노드 시작"
        )
        self.get_logger().info(
            f"goal frame: {self.odom_frame}"
        )

    def odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.current_pose = (
            float(position.x),
            float(position.y),
            quaternion_to_yaw(orientation),
        )
        self.last_odom_time = self.get_clock().now()
        self.waiting_for_odom_logged = False

    def goal_callback(self, message):
        frame_id = message.header.frame_id.strip("/")
        allowed_frames = {
            "",
            "odom",
            self.odom_frame,
            self.odom_frame.strip("/"),
        }

        if frame_id not in allowed_frames:
            self.get_logger().error(
                f"지원하지 않는 goal frame: {message.header.frame_id}. "
                f"{self.odom_frame}을 사용하세요."
            )
            return

        self.goal = (
            float(message.pose.position.x),
            float(message.pose.position.y),
        )
        self.goal_reached_publisher.publish(Bool(data=False))
        self.get_logger().info(
            f"새 목표 수신: x={self.goal[0]:.2f}, "
            f"y={self.goal[1]:.2f}"
        )

    def publish_stop(self):
        self.cmd_vel_publisher.publish(Twist())

    def odom_is_fresh(self):
        if self.last_odom_time is None:
            return False

        age = (
            self.get_clock().now() - self.last_odom_time
        ).nanoseconds / 1_000_000_000
        return age <= self.odom_timeout

    def control_step(self):
        if self.goal is None:
            return

        if self.current_pose is None or not self.odom_is_fresh():
            self.publish_stop()
            if not self.waiting_for_odom_logged:
                self.get_logger().warning(
                    "odom을 기다리는 중입니다. Forklift를 정지합니다."
                )
                self.waiting_for_odom_logged = True
            return

        current_x, current_y, current_yaw = self.current_pose
        goal_x, goal_y = self.goal

        command = calculate_velocity_command(
            current_x,
            current_y,
            current_yaw,
            goal_x,
            goal_y,
            linear_gain=self.linear_gain,
            angular_gain=self.angular_gain,
            min_linear_speed=self.min_linear_speed,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            distance_tolerance=self.distance_tolerance,
        )

        if command.reached:
            self.publish_stop()
            self.goal_reached_publisher.publish(Bool(data=True))
            self.get_logger().info(
                f"목표 도착: x={goal_x:.2f}, y={goal_y:.2f}"
            )
            self.goal = None
            return

        twist = Twist()
        twist.linear.x = command.linear
        twist.angular.z = command.angular
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()

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
