import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    desc_pkg = get_package_share_directory('ackermann_robot_description')

    urdf_file = os.path.join(desc_pkg, 'urdf', 'ackermann_robot.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])

    world_file = 'obstacles.world'
    world = os.path.join(desc_pkg, 'worlds', world_file)

    return LaunchDescription([
        GroupAction([
            ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
                output='screen'
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen'
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_ackermann_robot',
                arguments=['-entity', 'ackermann_robot', '-topic', 'robot_description'],
                output='screen'
            ),
        ]),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=['rear_left_wheel_velocity_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=['rear_right_wheel_velocity_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=['front_left_steering_position_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=['front_right_steering_position_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),
    ])
