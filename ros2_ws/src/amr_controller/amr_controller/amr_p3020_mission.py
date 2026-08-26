"""AMR mission orchestration for cargo delivery and P3020 handoff.

Sequence:
1) local IW Hub control rotates +90, drives to cargo, and lifts it.
2) Nav2 starts only after PICKUP_DONE.
3) delivery Nav2 success triggers P3020 PickPlace action.
4) P3020 success triggers Nav2 return to cargo area.
5) local controller restores IW Hub to the cargo dock pose and lowers lift.
6) after cargo pose verification, local controller returns IW Hub to spawn.

For AMR-only testing before the P3020 server is ready:
    simulate_p3020:=true
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

        # rviz2 Publish Point real measurement -- matches
        # p3020_mission_agent.py's AMR_DELIVERY_POSE_WORLD. AMR parks here,
        # facing 0 deg (toward the arm), so it is within arm #1's 2 m reach.
        self.declare_parameter("delivery_x", 1.7009891271591187)
        self.declare_parameter("delivery_y", -1.369241714477539)
        self.declare_parameter("delivery_yaw", 0.0)

        # Approximate return waypoint near the real cargo dock (9, -3) --
        # iw_hub_mission_agent.py's local RETURN_ALIGN_X/RETURN_ENTER_HOME
        # steps do the precise re-docking after Nav2 gets it this close.
        self.declare_parameter("return_x", 9.0)
        self.declare_parameter("return_y", -3.5)
        self.declare_parameter(
            "return_yaw",
            math.radians(90.0),
        )

        self.declare_parameter(
            "p3020_action_name",
            "/p3020/pick_place",
        )
        self.declare_parameter("simulate_p3020", True)
        self.declare_parameter("simulated_p3020_duration", 3.0)
        self.declare_parameter("object_id", "box")

        # place_x/y default to the real conveyor drop point -- was (0,0)
        # (unset), which put the box at the world origin instead of on the
        # conveyor.
        _pose_defaults = {
            "pickup": (0.0, 0.0, 0.0),
            "place": (-0.5, 0.0, 0.0),
        }
        for prefix in ("pickup", "place"):
            default_x, default_y, default_z = _pose_defaults[prefix]
            for field, default in (
                ("x", default_x),
                ("y", default_y),
                ("z", default_z),
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
            str(self.get_parameter("p3020_action_name").value),
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

        self.state = "WAIT_READY"
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

        if purpose == "DELIVERY":
            self.get_logger().info(
                "IW Hub destination arrived: sending P3020 action"
            )
            self._set_state("P3020_START")
        elif purpose == "RETURN_APPROACH":
            self._set_state("REQUEST_RETURN_DOCK")
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
                f"P3020 PickPlace complete: {result.message}"
            )
            self._set_state("NAV_TO_RETURN")
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
        if self.state == "WAIT_READY":
            if (
                self.navigate_client.server_is_ready()
                and self.pickup_state != "UNKNOWN"
            ):
                self._set_state("REQUEST_PICKUP")
            return

        if self.state == "REQUEST_PICKUP":
            self._publish_pickup_command("PICKUP")

            if self.pickup_state == "PICKUP_DONE":
                self.get_logger().info(
                    "cargo lift confirmed; Nav2 starts now"
                )
                self._set_state("NAV_TO_DELIVERY")
            elif self.pickup_state == "ERROR":
                self._fail(
                    "Isaac local pickup/lift controller reported ERROR"
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
                    "P3020 simulated: action/OpenCV server not connected"
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
                self.get_logger().info(
                    "simulated P3020 PickPlace complete"
                )
                self._set_state("NAV_TO_RETURN")
            return

        if self.state in {"P3020_GOAL_SENT", "P3020_WORKING"}:
            return

        if self.state == "NAV_TO_RETURN":
            self._send_nav_goal(
                "RETURN_APPROACH",
                float(self.get_parameter("return_x").value),
                float(self.get_parameter("return_y").value),
                float(self.get_parameter("return_yaw").value),
            )
            return

        if self.state == "REQUEST_RETURN_DOCK":
            self._publish_pickup_command("RETURN_DOCK")

            if self.pickup_state == "RETURN_DOCK_DONE":
                self.get_logger().info(
                    "IW Hub restored to cargo dock: "
                    "x=9, y=-3, yaw=90 deg"
                )
                self._set_state("REQUEST_LOWER")
            elif self.pickup_state == "ERROR":
                self._fail(
                    "Isaac return docking controller reported ERROR"
                )
            return

        if self.state == "REQUEST_LOWER":
            self._publish_pickup_command("LOWER")

            if self.pickup_state == "LOWER_DONE":
                self.get_logger().info(
                    "lift down complete; returning IW Hub to spawn"
                )
                self._set_state("REQUEST_RETURN_SPAWN")
            elif self.pickup_state == "ERROR":
                self._fail(
                    "Isaac lift-down/cargo pose verification ERROR"
                )
            return

        if self.state == "REQUEST_RETURN_SPAWN":
            self._publish_pickup_command("RETURN_SPAWN")

            if self.pickup_state == "SPAWN_DONE":
                self._set_state("COMPLETE")
            elif self.pickup_state == "ERROR":
                self._fail(
                    "Isaac spawn return controller reported ERROR"
                )
            return

        if self.state == "COMPLETE":
            self.get_logger().info(
                "AMR mission complete: cargo restored and IW Hub returned "
                "to spawn (9, -6, yaw=0 deg)",
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
