import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Paths to packages
    desc_pkg   = get_package_share_directory('ackermann_robot_description')
    nav_pkg    = get_package_share_directory('ackermann_robot_navigation')
    control_pkg= get_package_share_directory('ackermann_control_bridge')

    # URDF xacro
    urdf_file = os.path.join(desc_pkg, 'urdf', 'ackermann_robot.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])

    # Params
    world_file = 'nav2_tutorial.world.sdf'
    world = os.path.join(desc_pkg, 'worlds', world_file)

    # ros2_control configuration
    ros2_control_yaml = os.path.join(desc_pkg, 'config', 'ros2_control.yaml')

    return LaunchDescription([
        # 1) Launch Gazebo + Spawn Robot
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
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
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
                arguments=['-topic', 'robot_description', '-entity', 'ackermann_robot'],
                output='screen'
            ),
        ]),

        # 2) Launch ros2_control_node + Controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_desc}, ros2_control_yaml, {'use_sim_time': True}],
            output='screen'
        ),
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

        # 3) Launch Ackermann Bridge (delay slightly if needed)
        Node(
            package='ackermann_control_bridge',
            executable='ackermann_bridge_node',
            name='ackermann_bridge',
            output='screen'
        ),


    ])

