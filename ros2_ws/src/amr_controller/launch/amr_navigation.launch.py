from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="amr_a",
        description="Forklift ROS 2 namespace",
    )
    params_argument = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("amr_controller"),
                "config",
                "navigation.yaml",
            ]
        ),
        description="Goal navigator parameter file",
    )

    navigator_node = Node(
        package="amr_controller",
        executable="goal_navigator",
        namespace=LaunchConfiguration("namespace"),
        name="goal_navigator",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(
        [
            namespace_argument,
            params_argument,
            navigator_node,
        ]
    )
