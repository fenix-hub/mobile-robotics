import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    nav_pkg = get_package_share_directory('ackermann_robot_navigation')

    return LaunchDescription([
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', os.path.join(nav_pkg, 'rviz', 'config_navigation.rviz')]
                ),
            ]
        ),
    ])
