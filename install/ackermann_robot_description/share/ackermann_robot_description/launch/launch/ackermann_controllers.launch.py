#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('ackermann_robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ackermann_robot.urdf.xacro')
    ros2_control_yaml = os.path.join(pkg_share, 'config', 'ros2_control.yaml')

    # Process the URDF
    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
    

    
        # Spawn each controller in turn
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['rear_left_wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['rear_right_wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['front_left_steering_position_controller', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['front_right_steering_position_controller', '--controller-manager', '/controller_manager'],
        ),
    ])

