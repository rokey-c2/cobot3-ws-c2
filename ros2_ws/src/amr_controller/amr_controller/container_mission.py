"""Lift a loaded container, navigate, and place it at the destination."""

import sys

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from amr_controller.mission_policy import linear_ramp


class ContainerMission(Node):
    """One-shot IW Hub lift, NavigateToPose, and lower sequence."""

    def __init__(self):
        super().__init__("container_mission")
        self.declare_parameter("goal_x", 6.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("lift_height", 0.30)
        self.declare_parameter("lift_duration", 4.0)
        self.declare_parameter("settle_duration", 1.0)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.lift_height = float(
            self.get_parameter("lift_height").value
        )
        self.lift_duration = float(
            self.get_parameter("lift_duration").value
        )
        self.settle_duration = float(
            self.get_parameter("settle_duration").value
        )
        if self.lift_height <= 0.0:
            raise ValueError("lift_height must be positive")
        if self.lift_duration <= 0.0:
            raise ValueError("lift_duration must be positive")
        if self.settle_duration < 0.0:
            raise ValueError("settle_duration cannot be negative")

        self.lift_publisher = self.create_publisher(
            JointState, "lift_cmd", 10
        )
        self.navigation_publisher = self.create_publisher(
            Bool, "navigation_enabled", 10
        )
        self.state_publisher = self.create_publisher(
            String, "container_mission_state", 10
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        self.state = "WAITING_FOR_NAV2"
        self.phase_started_at = None
        self.exit_code = 0
        self._failure_reason = None
        self._shutdown_at = None
        self.timer = self.create_timer(0.05, self._tick)
        self._publish_state()
        self.get_logger().info(
            "mission: lift container -> navigate around obstacle -> "
            f"place at ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _publish_state(self):
        message = String()
        message.data = self.state
        self.state_publisher.publish(message)

    def _set_state(self, state):
        self.state = state
        self.phase_started_at = self._now_seconds()
        self._publish_state()
        self.get_logger().info(f"mission state: {state}")

    def _publish_navigation_enabled(self, enabled):
        message = Bool()
        message.data = bool(enabled)
        self.navigation_publisher.publish(message)

    def _publish_lift(self, position):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["lift_joint"]
        message.position = [float(position)]
        self.lift_publisher.publish(message)

    def _send_navigation_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.w = 1.0
        future = self.navigate_client.send_goal_async(
            goal, feedback_callback=self._navigation_feedback
        )
        future.add_done_callback(self._goal_response)
        self._set_state("SENDING_GOAL")

    def _goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - ROS transport error
            self._begin_failure(f"goal request failed: {error}")
            return
        if not goal_handle.accepted:
            self._begin_failure("navigation goal was rejected")
            return

        self._set_state("NAVIGATING_WITH_CONTAINER")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result)

    def _navigation_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        remaining = float(feedback.distance_remaining)
        self.get_logger().info(
            f"container transport remaining: {remaining:.2f} m",
            throttle_duration_sec=2.0,
        )

    def _navigation_result(self, future):
        try:
            result = future.result()
        except Exception as error:  # pragma: no cover - ROS transport error
            self._begin_failure(f"navigation result failed: {error}")
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._set_state("LOWERING_AT_DESTINATION")
            return
        self._begin_failure(
            f"navigation ended with status {result.status}"
        )

    def _begin_failure(self, reason):
        self.exit_code = 1
        self._failure_reason = reason
        self.get_logger().error(reason)
        self._publish_navigation_enabled(False)
        self._set_state("LOWERING_AFTER_FAILURE")

    def _tick(self):
        now = self._now_seconds()

        if self.state == "WAITING_FOR_NAV2":
            if self.navigate_client.server_is_ready():
                self._publish_navigation_enabled(True)
                self._set_state("LIFTING_CONTAINER")
            return

        elapsed = now - self.phase_started_at
        if self.state == "LIFTING_CONTAINER":
            position, complete = linear_ramp(
                0.0,
                self.lift_height,
                elapsed,
                self.lift_duration,
            )
            self._publish_lift(position)
            if complete:
                self._set_state("LIFT_SETTLING")
            return

        if self.state == "LIFT_SETTLING":
            self._publish_lift(self.lift_height)
            if elapsed >= self.settle_duration:
                self._send_navigation_goal()
            return

        if self.state in {"SENDING_GOAL", "NAVIGATING_WITH_CONTAINER"}:
            self._publish_lift(self.lift_height)
            return

        if self.state in {
            "LOWERING_AT_DESTINATION",
            "LOWERING_AFTER_FAILURE",
        }:
            position, complete = linear_ramp(
                self.lift_height,
                0.0,
                elapsed,
                self.lift_duration,
            )
            self._publish_lift(position)
            if complete:
                self._publish_navigation_enabled(False)
                final_state = (
                    "COMPLETED"
                    if self.exit_code == 0
                    else "FAILED_SAFE_LOWERED"
                )
                self._set_state(final_state)
                self._shutdown_at = now + 0.5
            return

        if self.state in {"COMPLETED", "FAILED_SAFE_LOWERED"}:
            self._publish_lift(0.0)
            if now >= self._shutdown_at:
                if self.exit_code == 0:
                    self.get_logger().info(
                        "container placed; mission complete"
                    )
                else:
                    self.get_logger().error(
                        "mission failed and lift was lowered safely: "
                        f"{self._failure_reason}"
                    )
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ContainerMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish_navigation_enabled(False)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()

