from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Get package directories
    nav_share = get_package_share_directory('nav')
    yolo_share = get_package_share_directory('yolo_realsense')
    
    return LaunchDescription([
        # Launch RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(yolo_share, 'launch', 'realsense_launch.py'),
            ])
        ),
        
        # Launch YOLO processor
        Node(
            package='yolo_realsense',
            executable='yolo_processor',
            name='yolo_processor',
            output='log'
        ),
        
        # Launch joystick + STM32 bridge (controller system)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav_share, 'launch', 'controller_launch.py')
            ])
        ),

        # Launch tmux monitoring of topics
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                    'sleep 8 && ' + os.path.join(nav_share, 'launch', 'monitor.sh')],
            output='screen',
            shell=False
        )

    ])