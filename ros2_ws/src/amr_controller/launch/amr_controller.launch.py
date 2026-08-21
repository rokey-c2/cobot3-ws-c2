from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Forklift AMR 테스트 노드를 실행한다."""

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

    return LaunchDescription(
        [
            namespace_argument,
            amr_controller_node,
        ]
    )
