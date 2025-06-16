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
    nav2_params = os.path.join(nav_pkg, 'params', 'nav2_params.yaml')
    map_yaml    = os.path.join(nav_pkg, 'maps', 'blank_map.yaml')

    # ros2_control configuration
    ros2_control_yaml = os.path.join(desc_pkg, 'config', 'ros2_control.yaml')

    return LaunchDescription([
        # 1) Launch Gazebo + Spawn Robot
        GroupAction([
            ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', os.path.join(desc_pkg, 'worlds', 'empty.world')],
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

        # 4) Launch Nav2
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'recoveries_server', 'bt_navigator']}
            ]
        ),

        # 5) (Optional) RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(nav_pkg, 'rviz', 'nav2_default_view.rviz')]
        ),
    ])

