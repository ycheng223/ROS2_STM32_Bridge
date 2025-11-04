from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav',
            namespace='nav',
            name = 'nav_stm32',
            executable='stm32_bridge_node', # needs to match name of executable in setup.py
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baud_rate': 115200}
            ]
        )
    ])