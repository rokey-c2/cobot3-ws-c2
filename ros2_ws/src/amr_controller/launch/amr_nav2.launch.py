from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
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
    autostart_argument = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically activate Nav2 lifecycle nodes",
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

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        namespace=namespace,
        name="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        namespace=namespace,
        name="smoother_server",
        output="screen",
        parameters=[configured_params],
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        namespace=namespace,
        name="planner_server",
        output="screen",
        parameters=[configured_params],
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        namespace=namespace,
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        namespace=namespace,
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        namespace=namespace,
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params],
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        namespace=namespace,
        name="velocity_smoother",
        output="screen",
        parameters=[configured_params],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        namespace=namespace,
        name="collision_monitor",
        output="screen",
        parameters=[configured_params],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=namespace,
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                    "collision_monitor",
                ],
            }
        ],
    )

    return LaunchDescription(
        [
            namespace_argument,
            params_argument,
            sim_time_argument,
            autostart_argument,
            static_map,
            velocity_mux,
            map_to_odom,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            collision_monitor,
            lifecycle_manager,
        ]
    )
