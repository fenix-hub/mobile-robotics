#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ackermann_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ackermann_robot.urdf.xacro')

    # Process the Xacro to produce robot_description
    robot_description = Command(['xacro ', urdf_file])

    # Use the obstacles world instead of empty world
    world_file = os.path.join(pkg_share, 'worlds', 'obstacles.world')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),

        # Publish joint states from the URDF
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Robot State Publisher (to publish TF from urdf)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Ensure robot spawns at ground level
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_ackermann_robot',
            arguments=['-entity', 'ackermann_robot', '-topic', '/obot_description', '-x', '0', '-y', '0', '-z', '0'],
            output='screen'
        ),
    ])
