import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ackermann_robot_navigation')
    default_map_yaml = os.path.join(pkg_share, 'maps', 'blank_map.yaml')   # If you created one
    default_params_file = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    default_bt_xml = os.path.join(pkg_share, 'params', 'navigate_w_replanning_and_recovery.xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'params',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use for all launched ackermann_control_bridge'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map')},
                        {'use_sim_time': True}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('params')]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('params')]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('params')]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[LaunchConfiguration('params')]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params')]
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
    ])

