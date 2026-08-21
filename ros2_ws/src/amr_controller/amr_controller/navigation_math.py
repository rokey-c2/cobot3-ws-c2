"""Forklift 목표점 추종에 사용하는 ROS 비의존 계산 함수."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VelocityCommand:
    linear: float
    angular: float
    distance: float
    heading_error: float
    reached: bool
    reversing: bool


def normalize_angle(angle):
    """각도를 -pi 이상 pi 미만으로 정규화한다."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def calculate_velocity_command(
    current_x,
    current_y,
    current_yaw,
    goal_x,
    goal_y,
    *,
    linear_gain,
    angular_gain,
    min_linear_speed,
    max_linear_speed,
    max_angular_speed,
    distance_tolerance,
):
    """현재 자세에서 목표점까지의 Forklift 속도 명령을 계산한다.

    Forklift는 제자리 회전을 할 수 없으므로 목표가 뒤쪽에 있으면
    후진 경로를 선택해 불필요하게 크게 회전하지 않는다.
    """

    delta_x = goal_x - current_x
    delta_y = goal_y - current_y
    distance = math.hypot(delta_x, delta_y)

    if distance <= distance_tolerance:
        return VelocityCommand(
            linear=0.0,
            angular=0.0,
            distance=distance,
            heading_error=0.0,
            reached=True,
            reversing=False,
        )

    target_heading = math.atan2(delta_y, delta_x)
    forward_error = normalize_angle(target_heading - current_yaw)
    reverse_error = normalize_angle(
        target_heading + math.pi - current_yaw
    )

    reversing = abs(reverse_error) < abs(forward_error)
    heading_error = reverse_error if reversing else forward_error
    direction = -1.0 if reversing else 1.0

    speed = clamp(
        linear_gain * distance,
        min_linear_speed,
        max_linear_speed,
    )

    # 방향 오차가 크면 천천히 움직이면서 최대 조향으로 방향을 맞춘다.
    limited_error = min(abs(heading_error), math.pi / 2.0)
    heading_scale = max(0.25, math.cos(limited_error) ** 2)
    linear_velocity = direction * speed * heading_scale

    angular_velocity = clamp(
        angular_gain * heading_error,
        -max_angular_speed,
        max_angular_speed,
    )

    return VelocityCommand(
        linear=linear_velocity,
        angular=angular_velocity,
        distance=distance,
        heading_error=heading_error,
        reached=False,
        reversing=reversing,
    )
