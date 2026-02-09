from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hamals_localization',
            executable='odom_tf_node',
            name='odom_tf_node',
            output='screen'
        )
    ])