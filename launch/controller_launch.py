from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Joystick driver
        Node(
            package='joy',
            namespace='nav',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 10.0
            }]
        ),

        # Joystick output converter (to ROS2 twist msgs)
        Node(
            package='teleop_twist_joy',
            namespace='nav',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'scale_linear.x': 1.0,
                'scale_linear_turbo.x': 1.2,
                'scale_angular.yaw': 1.0,
                'scale_angular_turbo.yaw': 1.2,
                'enable_button': 4,
                'enable_turbo_button': 5
            }],
            remappings=[
                ('joy', '/nav/joy'),
                ('cmd_vel', '/nav/cmd_vel')
            ]
        ),

        # STM32 bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav'),
                'launch', 'nav_launch.py')
            ])
        )
    ])