import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():
    # Paths to packages
    desc_pkg   = get_package_share_directory('ackermann_robot_description')
    nav_pkg    = get_package_share_directory('ackermann_robot_navigation')
    control_pkg= get_package_share_directory('ackermann_control_bridge')

    # URDF xacro
    urdf_file = os.path.join(desc_pkg, 'urdf', 'ackermann_robot.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])

    # Params
    world_file = 'nav2_tutorial.world.sdf'  # Changed from nav2_tutorial.world.sdf to empty.world
    world = os.path.join(desc_pkg, 'worlds', world_file)

    nav2_params = os.path.join(nav_pkg, 'params', 'nav2_params.yaml')
    # Add our new specialized planner and controller config
    nav2_planners_controllers = os.path.join(nav_pkg, 'params', 'nav2_planners_controllers.yaml')
    map_yaml    = os.path.join(nav_pkg, 'maps', 'blank_map.yaml')

    ekf_yaml = os.path.join(desc_pkg, 'config', 'ekf.yaml')
    # ros2_control configuration
    ros2_control_yaml = os.path.join(desc_pkg, 'config', 'ros2_control.yaml')

    # Path to the initial pose publisher script
    initial_pose_script = os.path.join(nav_pkg, 'scripts', 'publish_initial_pose.py')

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
            # Make sure robot_state_publisher is properly configured
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

        # Add a static transform publisher for map->odom until AMCL takes over
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # Add tf2_buffer_server to centralize TF buffering and reduce timing issues
        # Node(
        #     package='tf2_ros',
        #     executable='buffer_server',
        #     name='tf2_buffer_server',
        #     parameters=[
        #         {'use_sim_time': True},
        #         {'buffer_size': 120.0}
        #     ],
        #     output='screen'
        # ),

        # 2) Launch ros2_control_node + Controllers
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
                'base_frame': 'base_footprint',
                'publish_rate': 50.0,
            }]
        ),

        # # 5) Launch EKF for localization
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',  # Changed to match the name you're looking for
        #     output='screen',
        #     parameters=[ekf_yaml, {'use_sim_time': True}],
        # ),

        # 5) Launch Nav2
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
        # Update planner_server to use our specialized configuration
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),
        # Update controller_server to use our specialized configuration
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                nav2_params,
                {'use_sim_time': True},
                # Increase tf buffer to handle timing issues
                {'tf_buffer_duration': 10.0},
                {'odom_topic': '/odom'}
            ]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[
                nav2_params,
                {'use_sim_time': True},
                # Increase tf buffer to handle timing issues
                {'tf_buffer_duration': 10.0}
            ]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                nav2_params,
                {'use_sim_time': True},
                # Increase tf buffer to handle timing issues
                {'tf_buffer_duration': 10.0}
            ]
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

        # Wait for the lifecycle manager to activate all nodes before publishing initial pose
        TimerAction(
            period=8.0,  # Give time for all nodes to activate
            actions=[
                # Run the initial pose publisher script to set AMCL's initial pose
                ExecuteProcess(
                    cmd=['python3', initial_pose_script],
                    output='screen'
                )
            ]
        ),

        # 6) (Optional) RViz2
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

        # Uncomment TF tree diagnostics to help debug transform issues
        # Node(
        #     package='tf2_ros',
        #     executable='tf2_monitor',
        #     name='tf2_monitor',
        #     output='screen'
        # ),
    ])
