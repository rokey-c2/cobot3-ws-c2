from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )

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
                "nav2_params.yaml",
            ]
        ),
        description="Nav2 parameter file",
    )
    sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use Isaac Sim /clock",
    )

    static_map = Node(
        package="amr_controller",
        executable="static_map_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    velocity_mux = Node(
        package="amr_controller",
        executable="velocity_mux",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "navigation_enabled_on_start": False,
            }
        ],
        remappings=[("cmd_vel", "cmd_vel_safe")],
    )
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_amr_a_odom",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            PathJoinSubstitution([namespace, "odom"]),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
            # Nav2 launch 내부 PythonExpression이 이 값을 평가한다.
            # 소문자 false는 Python 변수로 해석돼 NameError가 발생한다.
            "use_composition": "False",
        }.items(),
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        namespace=namespace,
        name="collision_monitor",
        output="screen",
        parameters=[configured_params],
    )
    collision_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=namespace,
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["collision_monitor"],
            }
        ],
    )

    return LaunchDescription(
        [
            namespace_argument,
            params_argument,
            sim_time_argument,
            static_map,
            velocity_mux,
            map_to_odom,
            nav2,
            collision_monitor,
            collision_lifecycle_manager,
        ]
    )
