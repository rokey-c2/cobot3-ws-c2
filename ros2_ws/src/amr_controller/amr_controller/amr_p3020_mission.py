"""AMR mission: Nav2 -> pickup -> delivery -> P3020 -> return.

P3020 vision is intentionally omitted here. The existing PickPlace action is
called with fixed pickup/place poses supplied as ROS parameters.

For AMR-only testing before the P3020 teammate is ready, the default is
simulate_p3020:=true.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from logistics_interfaces.action import PickPlace
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def _yaw_quaternion(yaw):
    half = 0.5 * float(yaw)
    return math.sin(half), math.cos(half)


class AmrP3020Mission(Node):
    def __init__(self):
        super().__init__("amr_p3020_mission")

        self.declare_parameter("predock_x", 10.5)
        self.declare_parameter("predock_y", -0.50)
        self.declare_parameter("predock_yaw", math.radians(90.0))

        self.declare_parameter("delivery_x", 1.30104)
        self.declare_parameter("delivery_y", -0.06065)
        self.declare_parameter("delivery_yaw", 0.0)

        self.declare_parameter("home_x", 10.581993103027344)
        self.declare_parameter("home_y", 0.3304140567779541)
        self.declare_parameter("home_yaw", 0.0)

        self.declare_parameter(
            "p3020_action_name",
            "/p3020/pick_place",
        )
        self.declare_parameter("simulate_p3020", True)
        self.declare_parameter("simulated_p3020_duration", 3.0)
        self.declare_parameter("object_id", "cargo_pod")

        for prefix in ("pickup", "place"):
            for field, default in (
                ("x", 0.0),
                ("y", 0.0),
                ("z", 0.0),
                ("qx", 0.0),
                ("qy", 0.0),
                ("qz", 0.0),
                ("qw", 1.0),
            ):
                self.declare_parameter(
                    f"{prefix}_{field}",
                    default,
                )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        self.p3020_client = ActionClient(
            self,
            PickPlace,
            str(
                self.get_parameter(
                    "p3020_action_name"
                ).value
            ),
        )

        self.pickup_command_pub = self.create_publisher(
            String,
            "/amr_a/pickup_command",
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            "/amr_a/mission_state",
            10,
        )
        self.create_subscription(
            String,
            "/amr_a/pickup_state",
            self._pickup_state_callback,
            10,
        )

        self.state = "WAIT_NAV2"
        self.pickup_state = "UNKNOWN"
        self.current_nav_purpose = None
        self.phase_started_at = self._now_seconds()
        self.goal_in_flight = False
        self.timer = self.create_timer(0.1, self._tick)

        self._publish_state()

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _set_state(self, state):
        if state != self.state:
            self.get_logger().info(
                f"mission state: {self.state} -> {state}"
            )
        self.state = state
        self.phase_started_at = self._now_seconds()
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_pickup_command(self, command):
        msg = String()
        msg.data = str(command)
        self.pickup_command_pub.publish(msg)

    def _pickup_state_callback(self, message):
        self.pickup_state = message.data.strip().upper()

    def _pose_from_parameters(self, prefix):
        pose = Pose()
        pose.position.x = float(
            self.get_parameter(f"{prefix}_x").value
        )
        pose.position.y = float(
            self.get_parameter(f"{prefix}_y").value
        )
        pose.position.z = float(
            self.get_parameter(f"{prefix}_z").value
        )
        pose.orientation.x = float(
            self.get_parameter(f"{prefix}_qx").value
        )
        pose.orientation.y = float(
            self.get_parameter(f"{prefix}_qy").value
        )
        pose.orientation.z = float(
            self.get_parameter(f"{prefix}_qz").value
        )
        pose.orientation.w = float(
            self.get_parameter(f"{prefix}_qw").value
        )
        return pose

    def _send_nav_goal(self, purpose, x, y, yaw):
        if self.goal_in_flight:
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        qz, qw = _yaw_quaternion(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.current_nav_purpose = purpose
        self.goal_in_flight = True

        future = self.navigate_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback,
        )
        future.add_done_callback(self._nav_goal_response)

        self.get_logger().info(
            f"Nav2 goal {purpose}: x={x:.3f}, y={y:.3f}, "
            f"yaw={math.degrees(yaw):.1f} deg"
        )

    def _nav_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.goal_in_flight = False
            self._fail(f"Nav2 goal request failed: {error}")
            return

        if not handle.accepted:
            self.goal_in_flight = False
            self._fail(
                f"Nav2 rejected {self.current_nav_purpose}"
            )
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"{self.current_nav_purpose} remaining "
            f"{float(feedback.distance_remaining):.2f} m",
            throttle_duration_sec=2.0,
        )

    def _nav_result(self, future):
        purpose = self.current_nav_purpose
        self.goal_in_flight = False

        try:
            wrapped = future.result()
        except Exception as error:
            self._fail(f"Nav2 result failed: {error}")
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(
                f"Nav2 {purpose} ended with status={wrapped.status}"
            )
            return

        if purpose == "PRE_DOCK":
            self._set_state("REQUEST_PICKUP")
        elif purpose == "DELIVERY":
            self._set_state("P3020_START")
        elif purpose == "HOME":
            self._set_state("COMPLETE")
        else:
            self._fail(
                f"unknown Nav2 purpose completed: {purpose}"
            )

    def _send_p3020_goal(self):
        goal = PickPlace.Goal()
        goal.object_id = str(
            self.get_parameter("object_id").value
        )
        goal.pickup_pose = self._pose_from_parameters("pickup")
        goal.place_pose = self._pose_from_parameters("place")

        future = self.p3020_client.send_goal_async(
            goal,
            feedback_callback=self._p3020_feedback,
        )
        future.add_done_callback(self._p3020_goal_response)
        self._set_state("P3020_GOAL_SENT")

    def _p3020_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self._fail(
                f"P3020 goal request failed: {error}"
            )
            return

        if not handle.accepted:
            self._fail("P3020 rejected PickPlace")
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._p3020_result)
        self._set_state("P3020_WORKING")

    def _p3020_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"P3020 {feedback.state} "
            f"{float(feedback.progress):.0f}%",
            throttle_duration_sec=1.0,
        )

    def _p3020_result(self, future):
        try:
            wrapped = future.result()
        except Exception as error:
            self._fail(f"P3020 result failed: {error}")
            return

        result = wrapped.result
        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and bool(result.success)
        ):
            self.get_logger().info(
                f"P3020 complete: {result.message}"
            )
            self._set_state("REQUEST_LOWER")
            return

        self._fail(
            "P3020 PickPlace failed: "
            f"status={wrapped.status}, "
            f"message={result.message}"
        )

    def _fail(self, reason):
        self.get_logger().error(str(reason))
        self._set_state("ERROR")

    def _tick(self):
        if self.state == "WAIT_NAV2":
            if self.navigate_client.server_is_ready():
                self._set_state("NAV_TO_PRE_DOCK")
            return

        if self.state == "NAV_TO_PRE_DOCK":
            self._send_nav_goal(
                "PRE_DOCK",
                float(self.get_parameter("predock_x").value),
                float(self.get_parameter("predock_y").value),
                float(self.get_parameter("predock_yaw").value),
            )
            return

        if self.state == "REQUEST_PICKUP":
            self._publish_pickup_command("PICKUP")

            if self.pickup_state == "PICKUP_DONE":
                self._set_state("NAV_TO_DELIVERY")
            elif self.pickup_state == "ERROR":
                self._fail(
                    "Isaac docking/lift controller reported ERROR"
                )
            return

        if self.state == "NAV_TO_DELIVERY":
            self._send_nav_goal(
                "DELIVERY",
                float(self.get_parameter("delivery_x").value),
                float(self.get_parameter("delivery_y").value),
                float(self.get_parameter("delivery_yaw").value),
            )
            return

        if self.state == "P3020_START":
            simulate = bool(
                self.get_parameter("simulate_p3020").value
            )

            if simulate:
                self.get_logger().warning(
                    "P3020 simulated: no vision/action server"
                )
                self._set_state("P3020_SIMULATING")
                return

            if self.p3020_client.server_is_ready():
                self._send_p3020_goal()
            else:
                self.get_logger().info(
                    "waiting for P3020 PickPlace ActionServer",
                    throttle_duration_sec=2.0,
                )
            return

        if self.state == "P3020_SIMULATING":
            duration = float(
                self.get_parameter(
                    "simulated_p3020_duration"
                ).value
            )
            if self._now_seconds() - self.phase_started_at >= duration:
                self._set_state("REQUEST_LOWER")
            return

        if self.state in {"P3020_GOAL_SENT", "P3020_WORKING"}:
            return

        if self.state == "REQUEST_LOWER":
            self._publish_pickup_command("LOWER")

            if self.pickup_state == "LOWER_DONE":
                self._set_state("NAV_HOME")
            elif self.pickup_state == "ERROR":
                self._fail(
                    "Isaac lift controller reported ERROR"
                )
            return

        if self.state == "NAV_HOME":
            self._send_nav_goal(
                "HOME",
                float(self.get_parameter("home_x").value),
                float(self.get_parameter("home_y").value),
                float(self.get_parameter("home_yaw").value),
            )
            return

        if self.state == "COMPLETE":
            self.get_logger().info(
                "AMR mission complete",
                throttle_duration_sec=5.0,
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = AmrP3020Mission()

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
