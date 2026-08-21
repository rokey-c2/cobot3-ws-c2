"""Foxglove PoseStamped 목표를 Nav2 NavigateToPose action으로 변환한다."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class FoxgloveGoalBridge(Node):
    """`goal_pose` 토픽을 같은 namespace의 Nav2 action으로 전달한다."""

    def __init__(self):
        super().__init__("foxglove_goal_bridge")

        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("server_check_period", 0.5)

        self.goal_frame = str(self.get_parameter("goal_frame").value).strip("/")
        server_check_period = float(
            self.get_parameter("server_check_period").value
        )

        if not self.goal_frame:
            raise ValueError("goal_frame must not be empty")
        if server_check_period <= 0.0:
            raise ValueError("server_check_period must be positive")

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self.goal_callback,
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "foxglove_goal_status",
            10,
        )

        self.pending_goal = None
        self.waiting_for_server_logged = False
        self.server_timer = self.create_timer(
            server_check_period,
            self.try_send_pending_goal,
        )

        namespace = self.get_namespace().rstrip("/")
        topic_prefix = namespace if namespace else ""
        self.get_logger().info(
            f"Foxglove goal bridge: {topic_prefix}/goal_pose -> "
            f"{topic_prefix}/navigate_to_pose"
        )

    def publish_status(self, text):
        self.status_publisher.publish(String(data=text))

    def goal_callback(self, message):
        """Foxglove가 발행한 목표를 검증하고 전송 대기열에 넣는다."""

        received_frame = message.header.frame_id.strip("/")
        if not received_frame:
            received_frame = self.goal_frame
            message.header.frame_id = self.goal_frame

        if received_frame != self.goal_frame:
            error = (
                f"rejected: frame '{message.header.frame_id}' is not "
                f"'{self.goal_frame}'"
            )
            self.get_logger().error(error)
            self.publish_status(error)
            return

        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            message.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = message
        self.pending_goal = goal
        self.publish_status("queued")
        self.try_send_pending_goal()

    def try_send_pending_goal(self):
        """Nav2 action server가 준비되면 가장 최근 목표를 전송한다."""

        if self.pending_goal is None:
            return

        if not self.action_client.server_is_ready():
            if not self.waiting_for_server_logged:
                self.get_logger().warning(
                    "Nav2 navigate_to_pose action server를 기다리는 중입니다."
                )
                self.waiting_for_server_logged = True
            return

        goal = self.pending_goal
        self.pending_goal = None
        self.waiting_for_server_logged = False

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        self.publish_status("sending")

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy future transports action errors here.
            text = f"send_failed: {error}"
            self.get_logger().error(text)
            self.publish_status(text)
            return

        if not goal_handle.accepted:
            self.get_logger().warning("Nav2가 Foxglove 목표를 거절했습니다.")
            self.publish_status("rejected")
            return

        self.get_logger().info("Nav2가 Foxglove 목표를 수락했습니다.")
        self.publish_status("accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
        except Exception as error:
            text = f"result_failed: {error}"
            self.get_logger().error(text)
            self.publish_status(text)
            return

        self.get_logger().info(f"Foxglove 목표 종료, action status={status}")
        self.publish_status(f"finished:{status}")


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveGoalBridge()

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
