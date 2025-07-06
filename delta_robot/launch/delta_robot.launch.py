from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('delta_robot'),
        'config',
        'delta_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='delta_robot',
            executable='kinematics',
            name='kinematics',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='delta_robot',
            executable='motion_planner',
            name='motion_planner',
            output='screen',
        ),
        Node(
            package='delta_robot',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
        ),
        Node(
            package='delta_robot',
            executable='range_scanner',
            name='range_scanner',
            output='screen',
        ),
        Node(
            package='delta_robot',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
        ),
    ])
