from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Create launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_imu = LaunchConfiguration('use_imu', default='true')
    use_joint_states = LaunchConfiguration('use_joint_states', default='true')
    odom_position_offset_x = LaunchConfiguration('odom_position_offset_x', default='10.0')
    odom_position_offset_y = LaunchConfiguration('odom_position_offset_y', default='10.0')

    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock if true'),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Use IMU data for orientation if true'),
        DeclareLaunchArgument(
            'use_joint_states',
            default_value='true',
            description='Use joint_states topic for wheel feedback if true'),
        DeclareLaunchArgument(
            'odom_position_offset_x',
            default_value='10.0',
            description='X offset for the odom frame position'),
        DeclareLaunchArgument(
            'odom_position_offset_y',
            default_value='10.0',
            description='Y offset for the odom frame position'),
    ]
    
    # Create our odometry node
    odometry_node = Node(
        package='ackermann_control_bridge',
        executable='ackermann_odometry_node',
        name='ackermann_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheelbase': 0.4,
            'track_width': 0.3,
            'wheel_radius': 0.05,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'footprint_frame': 'base_footprint',
            'publish_rate': 50.0,
            'use_imu': use_imu,
            'use_joint_states': use_joint_states,
            'imu_orientation_weight': 0.8,
            'wheel_encoder_weight': 0.7,
            'odom_position_offset_x': odom_position_offset_x,
            'odom_position_offset_y': odom_position_offset_y,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_yaw': 0.0
        }]
    )
    
    # Return a LaunchDescription object
    return LaunchDescription(args + [odometry_node])
