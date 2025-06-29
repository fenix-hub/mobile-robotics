import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    control_pkg = get_package_share_directory('ackermann_control_bridge')
    desc_pkg = get_package_share_directory('ackermann_robot_description')

    ekf_yaml = os.path.join(desc_pkg, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='ackermann_control_bridge',
            executable='ackermann_bridge_node',
            name='ackermann_bridge',
            output='screen'
        ),
        Node(
            package='ackermann_control_bridge',
            executable='ackermann_odometry_node',
            name='ackermann_odometry_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'wheelbase': 0.4,
                'track_width': 0.3,
                'wheel_radius': 0.05,
                'odom_frame': 'odom',
                'footprint_frame': 'base_footprint',
                'base_frame': 'base_link',
                'publish_rate': 50.0,
            }]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml, {'use_sim_time': True}],
        ),
    ])
