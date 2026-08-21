"""Isaac Sim ForkliftB와 ROS 2 토픽을 연결한다.

구독:
    /<namespace>/cmd_vel  (geometry_msgs/Twist)
    /<namespace>/fork_up (std_msgs/Bool)

발행:
    /<namespace>/odom     (nav_msgs/Odometry)
    /tf                   (tf2_msgs/TFMessage)
"""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


COMMAND_TIMEOUT_SECONDS = 0.5


class ForkliftRos2Adapter:
    """ROS 2 명령을 Forklift에 적용하고 시뮬레이션 자세를 발행한다."""

    def __init__(
        self,
        forklift_controller,
        namespace="amr_a",
    ):
        self.forklift_controller = forklift_controller
        self.robot = forklift_controller.robot

        if not rclpy.ok():
            rclpy.init(args=None)

        self.namespace = namespace.strip("/")
        self.odom_frame = f"{self.namespace}/odom"
        self.base_frame = f"{self.namespace}/base_link"

        self.node = Node(
            node_name="forklift_ros2_adapter",
            namespace=self.namespace,
        )

        self.cmd_vel_subscription = self.node.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        self.fork_subscription = self.node.create_subscription(
            Bool,
            "fork_up",
            self.fork_command_callback,
            10,
        )

        self.odom_publisher = self.node.create_publisher(
            Odometry,
            "odom",
            10,
        )
        # /tf는 namespace 아래가 아니라 ROS 전체 공용 토픽으로 발행한다.
        self.tf_publisher = self.node.create_publisher(
            TFMessage,
            "/tf",
            10,
        )

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.has_received_command = False
        self.last_command_time = time.monotonic()

        topic_prefix = f"/{self.namespace}"
        print("[ROS2] Forklift ROS2 Adapter 시작")
        print(f"[ROS2] 구독: {topic_prefix}/cmd_vel")
        print(f"[ROS2] 구독: {topic_prefix}/fork_up")
        print(f"[ROS2] 발행: {topic_prefix}/odom")
        print(
            f"[ROS2] TF: {self.odom_frame} -> "
            f"{self.base_frame}"
        )

    def cmd_vel_callback(self, message):
        """가장 최근의 주행 명령을 저장한다."""

        self.linear_velocity = float(message.linear.x)
        self.angular_velocity = float(message.angular.z)
        self.has_received_command = True
        self.last_command_time = time.monotonic()

    def fork_command_callback(self, message):
        """Fork 상승 또는 하강 명령을 적용한다."""

        if message.data:
            print("[ROS2] Fork 상승 명령 수신")
            self.forklift_controller.lift_up()
            return

        print("[ROS2] Fork 하강 명령 수신")
        self.forklift_controller.lift_down()

    @staticmethod
    def quaternion_to_yaw(w, x, y, z):
        """Isaac의 wxyz quaternion을 평면 yaw로 변환한다."""

        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    def publish_odometry(self):
        """Isaac World 자세를 odom과 TF로 발행한다."""

        position, orientation = self.robot.get_world_pose()
        linear_world = self.robot.get_linear_velocity()
        angular_world = self.robot.get_angular_velocity()

        # Isaac quaternion 순서는 w, x, y, z이다.
        quat_w = float(orientation[0])
        quat_x = float(orientation[1])
        quat_y = float(orientation[2])
        quat_z = float(orientation[3])

        yaw = self.quaternion_to_yaw(
            quat_w,
            quat_x,
            quat_y,
            quat_z,
        )

        # Isaac의 World 속도를 base_link 좌표계 속도로 변환한다.
        world_vx = float(linear_world[0])
        world_vy = float(linear_world[1])
        body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
        body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy

        stamp = self.node.get_clock().now().to_msg()

        odom_message = Odometry()
        odom_message.header.stamp = stamp
        odom_message.header.frame_id = self.odom_frame
        odom_message.child_frame_id = self.base_frame
        odom_message.pose.pose.position.x = float(position[0])
        odom_message.pose.pose.position.y = float(position[1])
        odom_message.pose.pose.position.z = float(position[2])
        odom_message.pose.pose.orientation.x = quat_x
        odom_message.pose.pose.orientation.y = quat_y
        odom_message.pose.pose.orientation.z = quat_z
        odom_message.pose.pose.orientation.w = quat_w
        odom_message.twist.twist.linear.x = body_vx
        odom_message.twist.twist.linear.y = body_vy
        odom_message.twist.twist.angular.z = float(angular_world[2])
        self.odom_publisher.publish(odom_message)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = quat_x
        transform.transform.rotation.y = quat_y
        transform.transform.rotation.z = quat_z
        transform.transform.rotation.w = quat_w
        self.tf_publisher.publish(TFMessage(transforms=[transform]))

    def update(self):
        """ROS 2 콜백, 안전 정지, odometry 발행을 한 프레임 처리한다."""

        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.publish_odometry()

        if not self.has_received_command:
            self.forklift_controller.stop()
            return

        command_age = time.monotonic() - self.last_command_time

        if command_age > COMMAND_TIMEOUT_SECONDS:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        self.forklift_controller.drive_from_cmd_vel(
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
        )

    def shutdown(self):
        """Forklift와 ROS 2 노드를 안전하게 종료한다."""

        self.forklift_controller.stop()
        self.node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("[ROS2] Forklift ROS2 Adapter 종료")
