import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Paths to packages
    desc_pkg   = get_package_share_directory('ackermann_robot_description')

    # URDF xacro
    urdf_file = os.path.join(desc_pkg, 'urdf', 'ackermann_robot.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])

    # Params
    world_file = 'obstacles.world'
    world = os.path.join(desc_pkg, 'worlds', world_file)

    # ros2_control configuration
    ros2_control_yaml = os.path.join(desc_pkg, 'config', 'ros2_control.yaml')
    ekf_yaml = os.path.join(desc_pkg, 'config', 'ekf.yaml')

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

        # Start controller spawners with a delay to ensure ros2_control_node is ready
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



        # 3) Launch Ackermann Bridge (delay slightly if needed)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ackermann_control_bridge',
                    executable='ackermann_bridge_node',
                    name='ackermann_bridge',
                    output='screen'
                ),
            ]
        ),

        # 4) Launch Odometry Node
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

        # 5) Launch EKF for localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml, {'use_sim_time': True}],
        ),
    ])
