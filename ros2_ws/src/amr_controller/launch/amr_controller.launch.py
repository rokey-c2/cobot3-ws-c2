from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Forklift AMR 시간 기반 테스트 노드와 명령 중재기를 실행한다."""

    namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="amr_a",
        description="Forklift ROS 2 namespace",
    )

    amr_controller_node = Node(
        package="amr_controller",
        executable="amr_controller",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
    )
    velocity_mux_node = Node(
        package="amr_controller",
        executable="velocity_mux",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"navigation_enabled_on_start": True}],
    )

    return LaunchDescription(
        [
            namespace_argument,
            amr_controller_node,
            velocity_mux_node,
        ]
    )
