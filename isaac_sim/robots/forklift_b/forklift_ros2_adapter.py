"""Isaac Sim ForkliftB와 ROS 2 주행, odometry, clock을 연결한다.

구독:
    /<namespace>/drive_cmd_vel  (geometry_msgs/Twist)
    /<namespace>/fork_up       (std_msgs/Bool)

발행:
    /<namespace>/odom          (nav_msgs/Odometry)
    /clock                     (rosgraph_msgs/Clock)
    /tf                        (tf2_msgs/TFMessage)
"""

import math
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


COMMAND_TIMEOUT_SECONDS = 0.5
LIDAR_MOUNT_X = 0.0
LIDAR_MOUNT_Y = 0.0
LIDAR_MOUNT_Z = 2.0


class ForkliftRos2Adapter:
    """ROS 2 명령을 Forklift에 적용하고 시뮬레이션 상태를 발행한다."""

    def __init__(
        self,
        forklift_controller,
        namespace="amr_a",
        simulation_time_provider=None,
    ):
        self.forklift_controller = forklift_controller
        self.robot = forklift_controller.robot
        self.simulation_time_provider = simulation_time_provider

        if not rclpy.ok():
            rclpy.init(args=None)

        self.namespace = namespace.strip("/")
        self.odom_frame = f"{self.namespace}/odom"
        self.base_frame = f"{self.namespace}/base_link"
        self.lidar_frame = f"{self.namespace}/lidar_link"

        self.node = Node(
            node_name="forklift_ros2_adapter",
            namespace=self.namespace,
        )

        self.cmd_vel_subscription = self.node.create_subscription(
            Twist,
            "drive_cmd_vel",
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
        self.clock_publisher = self.node.create_publisher(
            Clock,
            "/clock",
            10,
        )
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
        print(f"[ROS2] 구독: {topic_prefix}/drive_cmd_vel")
        print(f"[ROS2] 구독: {topic_prefix}/fork_up")
        print(f"[ROS2] 발행: {topic_prefix}/odom, /clock, /tf")

    def cmd_vel_callback(self, message):
        """가장 최근의 최종 주행 명령을 저장한다."""

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

    def simulation_stamp(self):
        """현재 Isaac 시뮬레이션 시간을 ROS Time으로 변환한다."""

        if self.simulation_time_provider is None:
            return self.node.get_clock().now().to_msg()

        seconds = max(0.0, float(self.simulation_time_provider()))
        whole_seconds = int(seconds)
        nanoseconds = int((seconds - whole_seconds) * 1_000_000_000)
        return Time(sec=whole_seconds, nanosec=nanoseconds)

    def publish_simulation_state(self):
        """Isaac 자세를 /clock, odom, TF로 같은 timestamp에 발행한다."""

        position, orientation = self.robot.get_world_pose()
        linear_world = self.robot.get_linear_velocity()
        angular_world = self.robot.get_angular_velocity()

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

        world_vx = float(linear_world[0])
        world_vy = float(linear_world[1])
        body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
        body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy

        stamp = self.simulation_stamp()
        self.clock_publisher.publish(Clock(clock=stamp))

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

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = float(position[0])
        odom_to_base.transform.translation.y = float(position[1])
        odom_to_base.transform.translation.z = float(position[2])
        odom_to_base.transform.rotation.x = quat_x
        odom_to_base.transform.rotation.y = quat_y
        odom_to_base.transform.rotation.z = quat_z
        odom_to_base.transform.rotation.w = quat_w

        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = stamp
        base_to_lidar.header.frame_id = self.base_frame
        base_to_lidar.child_frame_id = self.lidar_frame
        base_to_lidar.transform.translation.x = LIDAR_MOUNT_X
        base_to_lidar.transform.translation.y = LIDAR_MOUNT_Y
        base_to_lidar.transform.translation.z = LIDAR_MOUNT_Z
        base_to_lidar.transform.rotation.w = 1.0

        self.tf_publisher.publish(
            TFMessage(transforms=[odom_to_base, base_to_lidar])
        )

    def update(self):
        """ROS 콜백, 안전 정지, 시뮬레이션 상태를 한 프레임 처리한다."""

        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.publish_simulation_state()

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
