from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="amr_a",
                description="Forklift ROS 2 namespace",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8765",
                description="Foxglove WebSocket port",
            ),
            DeclareLaunchArgument(
                "address",
                default_value="0.0.0.0",
                description="Foxglove WebSocket bind address",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use Isaac Sim /clock",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "port": ParameterValue(port, value_type=int),
                        "address": address,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="amr_controller",
                executable="foxglove_goal_bridge",
                namespace=namespace,
                name="foxglove_goal_bridge",
                output="screen",
                parameters=[
                    {
                        "goal_frame": "map",
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
