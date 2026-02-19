from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('hamals_robot_description')

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'robot.urdf.xacro'
    )

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([

        # ROBOT STATE PUBLISHER
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),

        # ODOM TF NODE
        Node(
            package='hamals_localization',
            executable='odom_tf_node',
            parameters=[{
                'odom_topic': '/odom',
                'parent_frame': 'odom',
                'child_frame': 'base_footprint'
            }],
            output='screen'
        )

    ])