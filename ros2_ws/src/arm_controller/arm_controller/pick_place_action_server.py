"""P3020 PickPlace ActionServer.

amr_p3020_mission.py(AMR 미션 FSM)가 보내는 logistics_interfaces/action/
PickPlace 요청을 받아서 실제로 처리하는 서버. 시스템 파이썬(3.12, 일반
/opt/ros/jazzy)에서 돌아야 한다 -- Isaac Sim 내장 rclpy(파이썬 3.11)
프로세스 안에서는 이 커스텀 액션 타입으로 ActionServer를 만들 수 없다는
걸 직접 확인했다(둘 다 rmw_fastrtps_cpp를 쓰지만 각자 다른 빌드라
typesupport가 안 맞아서 "Type support not from this implementation"으로
실패함). 그래서 이 서버는 Isaac Sim 쪽(isaac_sim/robots/p3020/
p3020_mission_agent.py)과 표준 타입(std_msgs/String)만 쓰는 토픽 두
개로 통신한다:

    /arm_a/pick_place_command (String, JSON) -- 이 서버 -> Isaac Sim
        {"place_x": .., "place_y": .., "scan_hint_x": .., "scan_hint_y": ..}
    /arm_a/pick_place_status  (String)        -- Isaac Sim -> 이 서버
        "SCANNING" | "APPROACHING" | "GRASPING" | "MOVING" | "PLACING"
        | "DONE_SUCCESS" | "DONE_FAIL:<사유>"

실행:
    ros2 run arm_controller pick_place_action_server
"""

import json
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from logistics_interfaces.action import PickPlace

COMMAND_TOPIC = "/arm_a/pick_place_command"
STATUS_TOPIC = "/arm_a/pick_place_status"

# Isaac Sim 쪽 스캔/흡착/이동에 걸리는 시간을 감안한 넉넉한 타임아웃.
RESULT_TIMEOUT_SEC = 180.0

_PROGRESS_BY_STATE = {
    "SCANNING": 10.0,
    "APPROACHING": 30.0,
    "GRASPING": 50.0,
    "MOVING": 70.0,
    "PLACING": 90.0,
}


class PickPlaceActionServer(Node):
    def __init__(self):
        super().__init__("p3020_pick_place_action_server")

        # execute_callback은 아래에서 time.sleep()으로 상태를 폴링하는데,
        # 기본(단일 스레드) 실행기에서는 이게 "이미 스핀 중인 실행기를 또
        # 스핀하려는" 재진입 문제를 일으킨다(직접 재현: "Executor is already
        # spinning" 에러로 goal이 매번 실패함). MultiThreadedExecutor +
        # ReentrantCallbackGroup을 쓰면 execute_callback이 별도 스레드에서
        # 돌아서, 그동안 메인 스핀 스레드가 /arm_a/pick_place_status 구독
        # 콜백을 계속 처리해줄 수 있다.
        callback_group = ReentrantCallbackGroup()

        self.command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.create_subscription(
            String, STATUS_TOPIC, self._on_status, 10,
            callback_group=callback_group,
        )
        self.latest_status = None

        self._action_server = ActionServer(
            self,
            PickPlace,
            "/p3020/pick_place",
            execute_callback=self._execute_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            f"ready: {COMMAND_TOPIC} -> Isaac Sim, {STATUS_TOPIC} <- Isaac Sim"
        )

    def _on_status(self, msg: String):
        self.latest_status = msg.data

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            f"PickPlace goal: object_id={goal.object_id} "
            f"pickup=({goal.pickup_pose.position.x:.3f}, {goal.pickup_pose.position.y:.3f}) "
            f"place=({goal.place_pose.position.x:.3f}, {goal.place_pose.position.y:.3f})"
        )

        command = {
            "place_x": float(goal.place_pose.position.x),
            "place_y": float(goal.place_pose.position.y),
            "scan_hint_x": float(goal.pickup_pose.position.x),
            "scan_hint_y": float(goal.pickup_pose.position.y),
        }
        self.latest_status = None
        msg = String()
        msg.data = json.dumps(command)
        self.command_pub.publish(msg)

        result = PickPlace.Result()
        feedback = PickPlace.Feedback()
        elapsed = 0.0
        poll_period = 0.2

        while elapsed < RESULT_TIMEOUT_SEC:
            time.sleep(poll_period)
            elapsed += poll_period

            status = self.latest_status
            if status is None:
                continue

            if status.startswith("DONE_SUCCESS"):
                goal_handle.succeed()
                result.success = True
                result.message = "PickPlace complete"
                return result

            if status.startswith("DONE_FAIL"):
                goal_handle.succeed()
                result.success = False
                result.message = status.partition(":")[2] or "PickPlace failed"
                return result

            feedback.state = status
            feedback.progress = _PROGRESS_BY_STATE.get(status, 0.0)
            goal_handle.publish_feedback(feedback)

        goal_handle.abort()
        result.success = False
        result.message = f"timed out after {RESULT_TIMEOUT_SEC:.0f}s waiting for Isaac Sim"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
