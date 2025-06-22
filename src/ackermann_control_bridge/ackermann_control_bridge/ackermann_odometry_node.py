#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Imu, LaserScan, Image, PointCloud2
from cv_bridge import CvBridge

import math
import numpy as np
import cv2
from typing import Tuple, Dict, Optional, List


class AckermannOdometryNode(Node):
    def __init__(self):
        super().__init__('ackermann_odometry_node')

        # Declare robot parameters
        self.declare_parameter('wheelbase', 0.4)
        self.declare_parameter('track_width', 0.3)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('footprint_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('imu_orientation_weight', 0.7)  # Weight for IMU orientation (0-1)
        self.declare_parameter('wheel_encoder_weight', 0.7)  # Weight for wheel encoder data (0-1)
        self.declare_parameter('lidar_weight', 0.5)  # Weight for LiDAR scan matching (0-1)
        self.declare_parameter('camera_weight', 0.3)  # Weight for visual odometry (0-1)
        self.declare_parameter('odom_position_offset_x', 0.0)  # Position offset for odom frame
        self.declare_parameter('odom_position_offset_y', 0.0)  # Position offset for odom frame
        self.declare_parameter('initial_pose_x', 0.0)  # Initial robot pose x
        self.declare_parameter('initial_pose_y', 0.0)  # Initial robot pose y
        self.declare_parameter('initial_pose_yaw', 0.0)  # Initial robot pose yaw

        # 'use_*' parameters to control which sensors are used
        self.declare_parameter('use_joint_states', True)  # Whether to use joint_states topic
        self.declare_parameter('use_imu', True)  # Whether to use IMU data for orientation
        self.declare_parameter('use_lidar', True)  # Whether to use LiDAR for scan matching
        self.declare_parameter('use_camera', False)  # Whether to use camera for visual odometry

        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.footprint_frame = self.get_parameter('footprint_frame').value
        self.use_joint_states = self.get_parameter('use_joint_states').value
        self.use_imu = self.get_parameter('use_imu').value
        self.use_lidar = self.get_parameter('use_lidar').value
        self.use_camera = self.get_parameter('use_camera').value
        self.imu_orientation_weight = self.get_parameter('imu_orientation_weight').value
        self.wheel_encoder_weight = self.get_parameter('wheel_encoder_weight').value
        self.lidar_weight = self.get_parameter('lidar_weight').value
        self.camera_weight = self.get_parameter('camera_weight').value
        self.odom_position_offset_x = self.get_parameter('odom_position_offset_x').value
        self.odom_position_offset_y = self.get_parameter('odom_position_offset_y').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Joint names (should match those in joint_states)
        self.rear_left_wheel_joint = 'rear_left_wheel_joint'
        self.rear_right_wheel_joint = 'rear_right_wheel_joint'
        self.front_left_steering_joint = 'front_left_steering_joint'
        self.front_right_steering_joint = 'front_right_steering_joint'
        
        # Set up odometry publisher with reliable QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        
        # Set up TF broadcaster for odom->base_footprint transformation
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up static TF broadcaster for fixed transforms
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # State variables for odometry
        initial_pose_x = self.get_parameter('initial_pose_x').value
        initial_pose_y = self.get_parameter('initial_pose_y').value
        initial_pose_yaw = self.get_parameter('initial_pose_yaw').value

        self.x = initial_pose_x
        self.y = initial_pose_y
        self.theta = initial_pose_yaw
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()
        
        # Sensor data storage
        self.wheel_vx = 0.0
        self.wheel_vth = 0.0
        self.imu_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w
        self.imu_angular_velocity = 0.0
        self.imu_linear_accel = [0.0, 0.0, 0.0]
        self.has_imu_data = False

        # LiDAR and camera data storage
        self.last_scan = None
        self.current_scan = None
        self.last_image = None
        self.current_image = None
        self.has_lidar_data = False
        self.has_camera_data = False
        self.lidar_dx = 0.0
        self.lidar_dy = 0.0
        self.lidar_dtheta = 0.0
        self.camera_dx = 0.0
        self.camera_dy = 0.0
        self.camera_dtheta = 0.0

        # Create CV bridge for image conversion
        self.cv_bridge = CvBridge()

        # Subscribe to /cmd_vel to estimate odometry (lowest priority, used as fallback)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            qos
        )
        
        # Store actual wheel states
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.left_steer_angle = 0.0
        self.right_steer_angle = 0.0
        self.using_wheel_data = False  # Set to True when we start receiving wheel data
        
        # Subscribe to joint states - ONLY option since velocity topics are not available
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.get_logger().info('Using /joint_states for odometry calculation')

        # Subscribe to IMU data if enabled
        if self.use_imu:
            self.imu_sub = self.create_subscription(
                Imu,
                '/imu',
                self.imu_callback,
                10
            )
            self.get_logger().info('Using IMU data for orientation and angular velocity')

        # Subscribe to LiDAR scan data if enabled
        if self.use_lidar:
            self.scan_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                10
            )
            self.get_logger().info('Using LiDAR scan data for scan matching odometry')

        # Subscribe to camera data if enabled
        if self.use_camera:
            self.image_sub = self.create_subscription(
                Image,
                '/depth_camera/image_raw',
                self.image_callback,
                10
            )
            self.get_logger().info('Using camera images for visual odometry')

        # Subscribe to initial pose (e.g., from RVIZ or localization)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        # Create timer for publishing odometry at a fixed rate
        self.timer = self.create_timer(1.0/publish_rate, self.publish_odometry)
        
        self.get_logger().info('Ackermann Odometry Node Started with multi-sensor fusion')

        # Publish static transforms for wheel joints
        self.setup_static_transforms()

    def joint_states_callback(self, msg: JointState):
        """Process data from the joint_states topic"""
        # Create maps for easier access to joint data
        position_map = {}
        velocity_map = {}
        
        for i, name in enumerate(msg.name):
            # Store position if available and not NaN
            if i < len(msg.position) and not math.isnan(msg.position[i]):
                position_map[name] = msg.position[i]
            
            # Store velocity if available and not NaN
            if i < len(msg.velocity) and not math.isnan(msg.velocity[i]):
                velocity_map[name] = msg.velocity[i]

        self.get_logger().debug(f'Joint States: {position_map}, {velocity_map}')

        # Extract wheel velocities
        if self.rear_left_wheel_joint in velocity_map:
            self.left_wheel_vel = velocity_map[self.rear_left_wheel_joint]
            self.using_wheel_data = True
        
        if self.rear_right_wheel_joint in velocity_map:
            self.right_wheel_vel = velocity_map[self.rear_right_wheel_joint]
            self.using_wheel_data = True
        
        # Extract steering angles
        if self.front_left_steering_joint in position_map:
            self.left_steer_angle = position_map[self.front_left_steering_joint]
        
        if self.front_right_steering_joint in position_map:
            self.right_steer_angle = position_map[self.front_right_steering_joint]
    
    def imu_callback(self, msg: Imu):
        """Process data from the IMU sensor"""
        # Extract orientation quaternion
        self.imu_orientation[0] = msg.orientation.x
        self.imu_orientation[1] = msg.orientation.y
        self.imu_orientation[2] = msg.orientation.z
        self.imu_orientation[3] = msg.orientation.w

        # Extract angular velocity (around z-axis for yaw)
        self.imu_angular_velocity = msg.angular_velocity.z

        # Extract linear acceleration
        self.imu_linear_accel[0] = msg.linear_acceleration.x
        self.imu_linear_accel[1] = msg.linear_acceleration.y
        self.imu_linear_accel[2] = msg.linear_acceleration.z

        self.has_imu_data = True

    def scan_callback(self, msg: LaserScan):
        """Process data from the LiDAR sensor"""
        # Store the previous scan and update current scan
        self.last_scan = self.current_scan
        self.current_scan = msg

        # Only proceed with scan matching if we have two consecutive scans
        if self.last_scan is not None and self.current_scan is not None:
            self.has_lidar_data = True

            # Perform scan matching to estimate motion
            dx, dy, dtheta = self.estimate_motion_from_scans(self.last_scan, self.current_scan)

            # Store the estimated motion
            self.lidar_dx = dx
            self.lidar_dy = dy
            self.lidar_dtheta = dtheta

    def image_callback(self, msg: Image):
        """Process data from the camera sensor"""
        try:
            # Convert ROS image to OpenCV format
            current_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Store the previous image and update current image
            self.last_image = self.current_image
            self.current_image = current_image

            # Only proceed with visual odometry if we have two consecutive images
            if self.last_image is not None and self.current_image is not None:
                self.has_camera_data = True

                # Perform visual odometry to estimate motion
                dx, dy, dtheta = self.estimate_motion_from_images(self.last_image, self.current_image)

                # Store the estimated motion
                self.camera_dx = dx
                self.camera_dy = dy
                self.camera_dtheta = dtheta
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def cmd_vel_callback(self, msg: Twist):
        # We'll use this if wheel encoder data is not available
        self.vx = msg.linear.x
        self.vy = 0.0  # Ackermann vehicles don't have lateral velocity
        self.vth = msg.angular.z
    
    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Process initial pose estimate (e.g., from RVIZ or localization system)"""
        if msg.header.frame_id == self.odom_frame or msg.header.frame_id == 'map':
            # Extract position
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y

            # Extract orientation as quaternion and convert to Euler yaw
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            # Convert quaternion to yaw (Euler angle around Z)
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self.theta = math.atan2(siny_cosp, cosy_cosp)

            self.get_logger().info(f'Updated initial pose: x={self.x}, y={self.y}, theta={self.theta}')

    def estimate_motion_from_scans(self, last_scan: LaserScan, current_scan: LaserScan) -> Tuple[float, float, float]:
        """
        Estimate motion between two consecutive laser scans using a simple scan matching algorithm.
        This is a simplified implementation and could be improved with more sophisticated algorithms.

        Returns:
            Tuple[float, float, float]: Estimated dx, dy, dtheta
        """
        # Simplified scan matching implementation
        # In a real implementation, you would use a more sophisticated algorithm
        # like ICP (Iterative Closest Point) or correlative scan matching

        # For now, return zero motion as a placeholder
        # In a real implementation, this would return the estimated motion between scans
        return 0.0, 0.0, 0.0

    def estimate_motion_from_images(self, last_image: np.ndarray, current_image: np.ndarray) -> Tuple[float, float, float]:
        """
        Estimate motion between two consecutive camera images using visual odometry techniques.
        This is a simplified implementation and could be improved with more sophisticated algorithms.

        Returns:
            Tuple[float, float, float]: Estimated dx, dy, dtheta
        """
        # Simplified visual odometry implementation
        # In a real implementation, you would use feature tracking, optical flow,
        # or other visual odometry techniques

        # For now, return zero motion as a placeholder
        # In a real implementation, this would return the estimated motion between images
        return 0.0, 0.0, 0.0

    def update_odometry(self) -> Tuple[float, float, float]:
        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time
        
        if dt <= 0.0:
            return self.x, self.y, self.theta
        
        # Calculate velocities from wheel data (if available)
        if self.using_wheel_data:
            # Safety check for NaN values in wheel velocities
            if math.isnan(self.left_wheel_vel) or math.isnan(self.right_wheel_vel):
                self.get_logger().warn("NaN detected in wheel velocities, using fallback values")
                self.left_wheel_vel = 0.0 if math.isnan(self.left_wheel_vel) else self.left_wheel_vel
                self.right_wheel_vel = 0.0 if math.isnan(self.right_wheel_vel) else self.right_wheel_vel

            # Safety check for NaN values in steering angles
            if math.isnan(self.left_steer_angle) or math.isnan(self.right_steer_angle):
                self.get_logger().warn("NaN detected in steering angles, using fallback values")
                self.left_steer_angle = 0.0 if math.isnan(self.left_steer_angle) else self.left_steer_angle
                self.right_steer_angle = 0.0 if math.isnan(self.right_steer_angle) else self.right_steer_angle

            # Calculate vehicle velocity from wheel velocities
            wheel_avg_velocity = (self.left_wheel_vel + self.right_wheel_vel) / 2.0
            linear_velocity = wheel_avg_velocity * self.wheel_radius
            
            # Calculate turning radius from steering angles
            steer_angle_avg = (self.left_steer_angle + self.right_steer_angle) / 2.0
            
            # For small steering angles, we can use the bicycle model approximation
            if abs(steer_angle_avg) < 1e-3:
                angular_velocity = 0.0
            else:
                # Bicycle model: angular_velocity = linear_velocity / turning_radius
                # turning_radius = wheelbase / tan(steering_angle)
                # Prevent division by zero or very small values
                try:
                    turning_radius = self.wheelbase / math.tan(steer_angle_avg)
                    angular_velocity = linear_velocity / turning_radius
                except (ZeroDivisionError, ValueError):
                    self.get_logger().warn("Error calculating turning radius, using zero angular velocity")
                    angular_velocity = 0.0

            # Store wheel-based velocity estimates
            self.wheel_vx = linear_velocity
            self.wheel_vth = angular_velocity

        # Initialize variables for fusion
        fused_vx = 0.0
        fused_vy = 0.0
        fused_vth = 0.0
        total_weight = 0.0

        # Incorporate wheel encoder data if available
        if self.using_wheel_data:
            fused_vx += self.wheel_vx * self.wheel_encoder_weight
            fused_vth += self.wheel_vth * self.wheel_encoder_weight
            total_weight += self.wheel_encoder_weight

        # Incorporate IMU data if available
        if self.has_imu_data:
            # Safety check for NaN values in IMU
            if math.isnan(self.imu_angular_velocity):
                self.get_logger().warn("NaN detected in IMU angular velocity, using fallback value")
                self.imu_angular_velocity = 0.0

            fused_vth += self.imu_angular_velocity * self.imu_orientation_weight
            total_weight += self.imu_orientation_weight

        # Incorporate LiDAR scan matching data if available
        if self.has_lidar_data and self.use_lidar:
            # Safety check for NaN values
            if math.isnan(self.lidar_dx) or math.isnan(self.lidar_dy) or math.isnan(self.lidar_dtheta):
                self.get_logger().warn("NaN detected in LiDAR scan matching, skipping")
            else:
                # Convert scan matching motion to velocities
                lidar_vx = self.lidar_dx / dt
                lidar_vy = self.lidar_dy / dt
                lidar_vth = self.lidar_dtheta / dt

                fused_vx += lidar_vx * self.lidar_weight
                fused_vy += lidar_vy * self.lidar_weight
                fused_vth += lidar_vth * self.lidar_weight
                total_weight += self.lidar_weight

        # Incorporate visual odometry data if available
        if self.has_camera_data and self.use_camera:
            # Safety check for NaN values
            if math.isnan(self.camera_dx) or math.isnan(self.camera_dy) or math.isnan(self.camera_dtheta):
                self.get_logger().warn("NaN detected in visual odometry, skipping")
            else:
                # Convert visual odometry motion to velocities
                camera_vx = self.camera_dx / dt
                camera_vy = self.camera_dy / dt
                camera_vth = self.camera_dtheta / dt

                fused_vx += camera_vx * self.camera_weight
                fused_vy += camera_vy * self.camera_weight
                fused_vth += camera_vth * self.camera_weight
                total_weight += self.camera_weight

        # If no sensor data is available, fall back to cmd_vel
        if total_weight < 0.01:
            self.vx = self.vx  # Keep current value from cmd_vel
            self.vy = 0.0      # Ackermann can't move sideways without sensor confirmation
            self.vth = self.vth  # Keep current value from cmd_vel
        else:
            # Normalize by total weight
            self.vx = fused_vx / total_weight
            self.vy = fused_vy / total_weight
            self.vth = fused_vth / total_weight

        # Safety check for NaN values in final velocities
        if math.isnan(self.vx) or math.isnan(self.vy) or math.isnan(self.vth):
            self.get_logger().error("NaN detected in final velocities, resetting to zero")
            self.vx = 0.0 if math.isnan(self.vx) else self.vx
            self.vy = 0.0 if math.isnan(self.vy) else self.vy
            self.vth = 0.0 if math.isnan(self.vth) else self.vth

        # Update pose using 2D kinematics, now including possible lateral movement from sensors
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.vth * dt
        
        # Update odometry state
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize angle to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        return self.x, self.y, self.theta
    
    def get_orientation_quaternion(self) -> Tuple[float, float, float, float]:
        """
        Get orientation quaternion, prioritizing IMU if available.
        Check for NaN values and provide valid quaternions.
        """
        if self.has_imu_data and self.imu_orientation_weight > 0.5:
            # Check for NaN values in IMU orientation
            if (math.isnan(self.imu_orientation[0]) or
                math.isnan(self.imu_orientation[1]) or
                math.isnan(self.imu_orientation[2]) or
                math.isnan(self.imu_orientation[3])):
                self.get_logger().warn("NaN detected in IMU quaternion, using calculated orientation instead")
                return self.euler_to_quaternion(0.0, 0.0, self.theta)

            # Normalize quaternion to ensure it's valid
            qx, qy, qz, qw = self.imu_orientation
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)

            # Check if normalization would cause division by zero
            if norm < 1e-10:
                self.get_logger().warn("Near-zero quaternion detected, using calculated orientation instead")
                return self.euler_to_quaternion(0.0, 0.0, self.theta)

            # Return normalized quaternion
            return (qx/norm, qy/norm, qz/norm, qw/norm)
        else:
            # Fall back to calculated orientation from odometry
            return self.euler_to_quaternion(0.0, 0.0, self.theta)

    def publish_odometry(self):
        # Update odometry from sensors
        x, y, theta = self.update_odometry()
        
        # Get orientation as quaternion, prioritizing IMU if available
        qx, qy, qz, qw = self.get_orientation_quaternion()

        # First, publish the transform from odom to base_footprint
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.footprint_frame

        # Apply offset to position the odom frame away from the robot
        transform.transform.translation.x = x - self.odom_position_offset_x
        transform.transform.translation.y = y - self.odom_position_offset_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        
        # Send the transform
        self.tf_broadcaster.sendTransform(transform)

        # Next, publish the odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.footprint_frame

        # Set the position (with offset)
        odom.pose.pose.position.x = x - self.odom_position_offset_x
        odom.pose.pose.position.y = y - self.odom_position_offset_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Set the velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Set covariance
        # If you don't have good covariance data, at least put reasonable defaults
        # on the diagonal. The values below are just examples.
        covariance_position = 0.1  # m^2
        covariance_orientation = 0.1  # rad^2
        covariance_velocity = 0.05  # (m/s)^2
        covariance_angular = 0.05  # (rad/s)^2
        
        # Position covariance [x, y, z, roll, pitch, yaw]
        odom.pose.covariance[0] = covariance_position  # x
        odom.pose.covariance[7] = covariance_position  # y
        odom.pose.covariance[14] = 1000000.0  # z (we don't estimate this)
        odom.pose.covariance[21] = 1000000.0  # roll (we don't estimate this)
        odom.pose.covariance[28] = 1000000.0  # pitch (we don't estimate this)
        odom.pose.covariance[35] = covariance_orientation  # yaw
        
        # Velocity covariance [vx, vy, vz, vroll, vpitch, vyaw]
        odom.twist.covariance[0] = covariance_velocity  # vx
        odom.twist.covariance[7] = 1000000.0  # vy (ackermann can't move sideways)
        odom.twist.covariance[14] = 1000000.0  # vz (we don't estimate this)
        odom.twist.covariance[21] = 1000000.0  # vroll (we don't estimate this)
        odom.twist.covariance[28] = 1000000.0  # vpitch (we don't estimate this)
        odom.twist.covariance[35] = covariance_angular  # vyaw
        
        # Publish the message
        self.odom_pub.publish(odom)
    
    def setup_static_transforms(self):
        """
        Publish static transforms for wheel joints to prevent NaN values in TF tree.
        These transforms ensure proper visualization in RViz2 even if the joint state publisher
        doesn't provide valid values.
        """
        # Create static transforms for wheel joints
        transforms = []

        # Create proper wheel quaternion for 90-degree rotation around Y-axis (pi/2)
        # Using the correct quaternion calculation for Y-axis rotation
        half = math.pi / 4.0  # Half of pi/2
        qx = 0.0
        qy = math.sin(half)  # sin(pi/4)
        qz = 0.0
        qw = math.cos(half)  # cos(pi/4)

        # Safety check - ensure the quaternion is normalized
        norm = math.sqrt(qy*qy + qw*qw)
        if norm > 0:
            qy /= norm
            qw /= norm

        # Rear left wheel transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'rear_left_wheel'
        transform.transform.translation.x = -self.wheelbase/2
        transform.transform.translation.y = self.track_width/2 + 0.04  # Adding wheel_width/2 + clearance
        transform.transform.translation.z = -0.05  # -chassis_height/2
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        transforms.append(transform)

        # Rear right wheel transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'rear_right_wheel'
        transform.transform.translation.x = -self.wheelbase/2
        transform.transform.translation.y = -(self.track_width/2 + 0.04)  # Adding wheel_width/2 + clearance
        transform.transform.translation.z = -0.05  # -chassis_height/2
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        transforms.append(transform)


        # Front left wheel transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'front_left_steering_link'
        transform.child_frame_id = 'front_left_wheel'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.04  # wheel_width/2 + clearance
        transform.transform.translation.z = -0.05  # -chassis_height/2
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        transforms.append(transform)

        # Front right wheel transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'front_right_steering_link'
        transform.child_frame_id = 'front_right_wheel'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = -0.04  # -(wheel_width/2 + clearance)
        transform.transform.translation.z = -0.05  # -chassis_height/2
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        transforms.append(transform)

        # Publish all static transforms at once
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info('Published static transforms for wheel joints with fixed quaternions')

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw) -> Tuple[float, float, float, float]:
        """
        Convert Euler angles to quaternion.

        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians

        Returns:
            Tuple of (qx, qy, qz, qw) representing the quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
