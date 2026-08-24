"""주행 명령 우선순위를 결정하는 ROS 비의존 정책."""


STOP_SOURCE = "stop"
MANUAL_SOURCE = "manual"
NAVIGATION_SOURCE = "navigation"


def select_velocity_source(
    *,
    emergency_stop,
    navigation_enabled,
    manual_age,
    navigation_age,
    command_timeout,
):
    """E-stop > 최신 수동 명령 > 활성 Nav2 명령 순서로 선택한다."""

    if emergency_stop:
        return STOP_SOURCE

    if manual_age is not None and manual_age <= command_timeout:
        return MANUAL_SOURCE

    if (
        navigation_enabled
        and navigation_age is not None
        and navigation_age <= command_timeout
    ):
        return NAVIGATION_SOURCE

    return STOP_SOURCE

