"""
ForkliftB Controller

지원 기능:
- 전진 및 후진
- 좌우 조향
- Fork 상승 및 하강
- 시간 기반 기본 동작 테스트
- ROS2 cmd_vel 명령 처리
"""

import math

import numpy as np

from isaacsim.core.utils.types import ArticulationAction


# ---------------------------------------------------------
# Forklift 모델 설정
# ---------------------------------------------------------

# 구동 바퀴 반지름, 단위 m
WHEEL_RADIUS = 0.15

# 앞바퀴와 뒷바퀴 사이의 거리, 단위 m
WHEEL_BASE = 1.20

# 최대 조향 각도, 단위 degree
MAX_STEERING_ANGLE = 35.0

# 기본 테스트 조향 각도
DEFAULT_STEERING_ANGLE = 25.0

# 모델이 반대 방향으로 회전하면 -1.0으로 변경
STEERING_DIRECTION = 1.0

# 시간 기반 테스트에서 사용할 바퀴 회전 속도
DEFAULT_DRIVE_SPEED = 5.0

# Fork를 전체 이동 범위의 40%까지 올림
DEFAULT_LIFT_RATIO = 0.4


class ForkliftController:
    """ForkliftB의 이동, 조향, Fork를 제어한다."""

    def __init__(self, forklift_robot):
        self.robot = forklift_robot

        # USD에서 확인한 Joint 이름
        self.lift_joint_name = "lift_joint"
        self.steering_joint_name = (
            "back_wheel_swivel"
        )
        self.drive_joint_name = "back_wheel_drive"

        # Joint 이름을 Joint 번호로 변환
        self.lift_joint_index = (
            self.get_joint_index(
                self.lift_joint_name
            )
        )

        self.steering_joint_index = (
            self.get_joint_index(
                self.steering_joint_name
            )
        )

        self.drive_joint_index = (
            self.get_joint_index(
                self.drive_joint_name
            )
        )

        # Fork Joint의 최소·최대 이동 범위 확인
        lift_properties = self.robot.dof_properties[
            self.lift_joint_index
        ]

        self.lift_min_position = float(
            lift_properties["lower"]
        )

        self.lift_max_position = float(
            lift_properties["upper"]
        )

        # 테스트 상태 중복 출력을 방지하는 변수
        self.current_test_state = None

        self.print_controller_info()

    # -----------------------------------------------------
    # Joint 정보
    # -----------------------------------------------------

    def get_joint_index(self, joint_name):
        """Joint 이름에 해당하는 번호를 반환한다."""

        if joint_name not in self.robot.dof_names:
            raise ValueError(
                f"Joint를 찾을 수 없습니다: {joint_name}"
            )

        return self.robot.dof_names.index(joint_name)

    def print_controller_info(self):
        """제어에 사용하는 Joint 정보를 출력한다."""

        print("\n[FORKLIFT CONTROLLER]")

        print(
            f"Drive Joint: "
            f"{self.drive_joint_name} "
            f"(index {self.drive_joint_index})"
        )

        print(
            f"Steering Joint: "
            f"{self.steering_joint_name} "
            f"(index {self.steering_joint_index})"
        )

        print(
            f"Lift Joint: "
            f"{self.lift_joint_name} "
            f"(index {self.lift_joint_index})"
        )

        print(
            f"Lift Range: "
            f"{self.lift_min_position:.3f} "
            f"~ {self.lift_max_position:.3f}"
        )

    # -----------------------------------------------------
    # 구동 바퀴 제어
    # -----------------------------------------------------

    def set_drive_speed(self, wheel_speed):
        """구동 바퀴의 회전 속도를 설정한다."""

        action = ArticulationAction(
            joint_velocities=np.array(
                [wheel_speed],
                dtype=np.float32,
            ),
            joint_indices=np.array(
                [self.drive_joint_index],
                dtype=np.int32,
            ),
        )

        self.robot.apply_action(action)

    def move_forward(
        self,
        speed=DEFAULT_DRIVE_SPEED,
    ):
        """Forklift를 직진시킨다."""

        self.set_steering_angle(0.0)
        self.set_drive_speed(speed)

    def move_backward(
        self,
        speed=DEFAULT_DRIVE_SPEED,
    ):
        """Forklift를 후진시킨다."""

        self.set_steering_angle(0.0)
        self.set_drive_speed(-speed)

    def stop(self):
        """Forklift를 정지시킨다."""

        self.set_drive_speed(0.0)

    # -----------------------------------------------------
    # 조향 제어
    # -----------------------------------------------------

    def set_steering_angle(self, angle_degrees):
        """조향각을 도 단위로 입력받아 설정한다."""

        safe_angle_degrees = float(
            np.clip(
                angle_degrees,
                -MAX_STEERING_ANGLE,
                MAX_STEERING_ANGLE,
            )
        )

        angle_radians = math.radians(
            safe_angle_degrees
        )

        action = ArticulationAction(
            joint_positions=np.array(
                [angle_radians],
                dtype=np.float32,
            ),
            joint_indices=np.array(
                [self.steering_joint_index],
                dtype=np.int32,
            ),
        )

        self.robot.apply_action(action)

    def turn_left(
        self,
        speed=DEFAULT_DRIVE_SPEED,
        steering_angle=DEFAULT_STEERING_ANGLE,
    ):
        """전진하면서 왼쪽으로 조향한다."""

        self.set_steering_angle(
            steering_angle
        )
        self.set_drive_speed(speed)

    def reverse_left(
        self,
        speed=DEFAULT_DRIVE_SPEED,
        steering_angle=DEFAULT_STEERING_ANGLE,
    ):
        """후진하면서 기존 회전 경로를 돌아간다."""

        self.set_steering_angle(
            steering_angle
        )
        self.set_drive_speed(-speed)

    # -----------------------------------------------------
    # Fork 제어
    # -----------------------------------------------------

    def set_lift_position(self, target_position):
        """Fork의 목표 높이를 설정한다."""

        safe_position = float(
            np.clip(
                target_position,
                self.lift_min_position,
                self.lift_max_position,
            )
        )

        action = ArticulationAction(
            joint_positions=np.array(
                [safe_position],
                dtype=np.float32,
            ),
            joint_indices=np.array(
                [self.lift_joint_index],
                dtype=np.int32,
            ),
        )

        self.robot.apply_action(action)

    def lift_up(
        self,
        lift_ratio=DEFAULT_LIFT_RATIO,
    ):
        """Fork를 지정된 비율만큼 상승시킨다."""

        safe_lift_ratio = float(
            np.clip(
                lift_ratio,
                0.0,
                1.0,
            )
        )

        lift_range = (
            self.lift_max_position
            - self.lift_min_position
        )

        target_position = (
            self.lift_min_position
            + lift_range * safe_lift_ratio
        )

        self.set_lift_position(
            target_position
        )

    def lift_down(self):
        """Fork를 가장 낮은 위치로 내린다."""

        self.set_lift_position(
            self.lift_min_position
        )

    # -----------------------------------------------------
    # 시간 기반 기본 동작 테스트
    # -----------------------------------------------------

    def run_basic_test(self, elapsed_time):
        """기본 이동과 Fork 동작을 테스트한다."""

        if elapsed_time < 3.0:
            self.print_test_state("1. 직진")
            self.move_forward()

        elif elapsed_time < 5.0:
            self.print_test_state("2. 좌회전")
            self.turn_left()

        elif elapsed_time < 8.0:
            self.print_test_state(
                "3. 목표 위치까지 직진"
            )
            self.move_forward()

        elif elapsed_time < 9.0:
            self.print_test_state(
                "4. 목표 위치 정지"
            )
            self.stop()

        elif elapsed_time < 12.0:
            self.print_test_state(
                "5. Fork 상승"
            )
            self.stop()
            self.lift_up()

        elif elapsed_time < 14.0:
            self.print_test_state(
                "6. Fork 상승 상태 유지"
            )
            self.stop()

        elif elapsed_time < 17.0:
            self.print_test_state(
                "7. Fork 하강"
            )
            self.stop()
            self.lift_down()

        elif elapsed_time < 20.0:
            self.print_test_state("8. 후진")
            self.move_backward()

        elif elapsed_time < 22.0:
            self.print_test_state(
                "9. 후진하면서 회전"
            )
            self.reverse_left()

        elif elapsed_time < 25.0:
            self.print_test_state(
                "10. 시작 위치까지 후진"
            )
            self.move_backward()

        else:
            self.print_test_state(
                "11. 시작 위치 복귀 완료"
            )
            self.stop()
            self.set_steering_angle(0.0)

    def print_test_state(self, new_state):
        """테스트 상태가 바뀔 때 한 번만 출력한다."""

        if self.current_test_state == new_state:
            return

        self.current_test_state = new_state

        print(
            f"[FORKLIFT TEST] {new_state}"
        )

    # -----------------------------------------------------
    # ROS2 cmd_vel 제어
    # -----------------------------------------------------

    def drive_from_cmd_vel(
        self,
        linear_velocity,
        angular_velocity,
    ):
        """
        ROS2 Twist 명령을 Forklift 명령으로 변환한다.

        linear_velocity:
            전진·후진 속도, 단위 m/s

        angular_velocity:
            회전 속도, 단위 rad/s
        """

        # Forklift는 제자리 회전을 지원하지 않는다.
        if abs(linear_velocity) < 0.01:
            self.stop()
            self.set_steering_angle(0.0)
            return

        # 차량 속도를 바퀴 회전 속도로 변환
        wheel_speed = (
            linear_velocity
            / WHEEL_RADIUS
        )

        # Bicycle Model을 이용해 조향각 계산
        steering_angle_radians = math.atan(
            WHEEL_BASE
            * angular_velocity
            / linear_velocity
        )

        steering_angle_degrees = math.degrees(
            steering_angle_radians
        )

        # 모델의 조향 방향 보정
        steering_angle_degrees *= (
            STEERING_DIRECTION
        )

        # 최대 조향각을 넘지 않도록 제한
        steering_angle_degrees = float(
            np.clip(
                steering_angle_degrees,
                -MAX_STEERING_ANGLE,
                MAX_STEERING_ANGLE,
            )
        )

        # 계산된 조향각과 바퀴 속도 적용
        self.set_steering_angle(
            steering_angle_degrees
        )

        self.set_drive_speed(
            wheel_speed
        )