import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch.actions import TimerAction

def generate_launch_description():
    nav_pkg = get_package_share_directory('ackermann_robot_navigation')

    nav2_params = os.path.join(nav_pkg, 'params', 'nav2_params.yaml')
    map_yaml = os.path.join(nav_pkg, 'maps', 'ackermann_steering_map.yaml')
    default_bt_xml_file = os.path.join(nav_pkg, 'bt', 'navigate_w_replanning_and_recovery.xml')
    initial_pose_script = os.path.join(nav_pkg, 'scripts', 'publish_initial_pose.py')

    rewritten_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={'default_bt_xml_filename': default_bt_xml_file},
        convert_types=True
    )

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}, {'use_sim_time': True}]
        ),
        # Add a static transform publisher for map->odom until AMCL takes over
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[rewritten_params]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[rewritten_params]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                rewritten_params,
                {'use_sim_time': True},
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
                rewritten_params,
                {'use_sim_time': True},
                {'tf_buffer_duration': 10.0}
            ]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                rewritten_params,
                {'use_sim_time': True},
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
    ])
