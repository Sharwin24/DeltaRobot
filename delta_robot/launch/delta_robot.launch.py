from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='delta_robot',
            executable='kinematics',
            name='kinematics',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory('delta_robot'),
                    'config',
                    'delta_config.yaml'
                ])
            ],
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
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('delta_robot_sensors'),
                    'launch',
                    'sensors.launch.xml'
                ])
            )
        )
    ])
