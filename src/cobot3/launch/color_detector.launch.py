from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument('input_topic', default_value='/rgb')
    output_topic_arg = DeclareLaunchArgument('output_topic', default_value='/detected_color')
    min_pixels_arg = DeclareLaunchArgument('min_pixels', default_value='200')

    node = Node(
        package='cobot3',
        executable='m0609_color_detector',
        name='color_detector_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'min_pixels': LaunchConfiguration('min_pixels'),
        }],
    )

    return LaunchDescription([input_topic_arg, output_topic_arg, min_pixels_arg, node])
