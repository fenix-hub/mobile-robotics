#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header

import math
import numpy as np
from typing import Tuple, Dict, Optional


class AckermannOdometryNode(Node):
    def __init__(self):
        super().__init__('ackermann_odometry_node')

        # Declare robot parameters
        self.declare_parameter('wheelbase', 0.4)  # Distance between front and rear axles
        self.declare_parameter('track_width', 0.3)  # Distance between wheel centers on an axle
        self.declare_parameter('wheel_radius', 0.05)  # Radius of the wheels
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')  # base_link is typically used by robot_state_publisher
        self.declare_parameter('footprint_frame', 'base_footprint')  # odometry is published relative to base_footprint
        self.declare_parameter('publish_rate', 50.0)  # Hz

        # Odometry fusion weights (0-1)
        self.declare_parameter('imu_orientation_weight', 0.7)
        self.declare_parameter('wheel_encoder_weight', 0.7)
        self.declare_parameter('cmd_vel_weight', 0.1)  # Lower weight for cmd_vel as it's not feedback

        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.footprint_frame = self.get_parameter('footprint_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.imu_orientation_weight = self.get_parameter('imu_orientation_weight').value
        self.wheel_encoder_weight = self.get_parameter('wheel_encoder_weight').value
        self.cmd_vel_weight = self.get_parameter('cmd_vel_weight').value

        # Initialize odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0  # Will remain 0 for ackermann drive
        self.vtheta = 0.0

        # Last known sensor data for odometry calculation
        self.last_joint_states_time: Optional[rclpy.time.Time] = None
        self.last_imu_yaw: Optional[float] = None
        self.last_cmd_vel_time: Optional[rclpy.time.Time] = None
        self.last_cmd_linear_x: float = 0.0
        self.last_cmd_angular_z: float = 0.0

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry publisher
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        )

        # Subscribers
        qos_profile_sensor_data = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Imu,
            '/imu',  # Adjust topic name if different in your setup
            self.imu_callback,
            qos_profile_sensor_data
        )

        # Timer for publishing odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odom)
        self.last_time = self.get_clock().now()
        self.get_logger().info('Ackermann Odometry Node has started!')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_linear_x = msg.linear.x
        self.last_cmd_angular_z = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()
        # self.get_logger().debug(f'Received cmd_vel: linear_x={msg.linear.x}, angular_z={msg.angular.z}')

    def joint_state_callback(self, msg: JointState):
        current_time = self.get_clock().now()
        dt_duration = current_time - self.last_time if self.last_joint_states_time is None else current_time - self.last_joint_states_time
        dt = dt_duration.nanoseconds / 1e9

        if dt <= 0:
            return

        # Assuming 'front_left_wheel_joint' and 'front_right_wheel_joint' are velocity controlled
        # And 'rear_left_wheel_joint', 'rear_right_wheel_joint' for rear wheel velocities
        # Or you might just use one front wheel and one rear wheel velocity for a simpler model.
        # This implementation assumes front wheel velocities are primary for linear_x.

        # Find velocities for relevant joints
        front_left_vel = 0.0
        front_right_vel = 0.0
        rear_left_vel = 0.0
        rear_right_vel = 0.0

        for i, name in enumerate(msg.name):
            if name == 'front_left_wheel_joint' and i < len(msg.velocity):
                front_left_vel = msg.velocity[i]
            elif name == 'front_right_wheel_joint' and i < len(msg.velocity):
                front_right_vel = msg.velocity[i]
            elif name == 'rear_left_wheel_joint' and i < len(msg.velocity):
                rear_left_vel = msg.velocity[i]
            elif name == 'rear_right_wheel_joint' and i < len(msg.velocity):
                rear_right_vel = msg.velocity[i]

        # Calculate linear velocity (vx) from wheel encoders
        # Simple average of front wheel velocities, converted to m/s
        linear_x_encoder = ((front_left_vel * self.wheel_radius) + (front_right_vel * self.wheel_radius)) / 2.0

        # Calculate angular velocity (vtheta) from wheel encoders (for ackermann this is more complex)
        # For simplicity, we'll assume a basic unicycle model approximation for rotation or rely on IMU for yaw
        # A more accurate Ackermann model would use steering joint angles.
        # If no steering joint is available, vtheta might be inferred from different wheel speeds for rotation,
        # but for pure ackermann it's usually 0 unless turning.
        angular_z_encoder = 0.0  # Default to 0 for ackermann, IMU or steering input is better for this.

        # Fuse with cmd_vel if recent and weighted
        fused_linear_x = (linear_x_encoder * self.wheel_encoder_weight)
        fused_angular_z = (angular_z_encoder * self.wheel_encoder_weight)

        if self.last_cmd_vel_time is not None and (
                current_time - self.last_cmd_vel_time).nanoseconds / 1e9 < 0.5:  # if cmd_vel is recent
            fused_linear_x += (self.last_cmd_linear_x * self.cmd_vel_weight)
            fused_angular_z += (self.last_cmd_angular_z * self.cmd_vel_weight)
            total_weight_linear = self.wheel_encoder_weight + self.cmd_vel_weight
            total_weight_angular = self.wheel_encoder_weight + self.cmd_vel_weight
            if total_weight_linear > 0:
                fused_linear_x /= total_weight_linear
            if total_weight_angular > 0:
                fused_angular_z /= total_weight_angular

        self.vx = fused_linear_x
        self.vtheta = fused_angular_z  # This will be primarily IMU or steering based in true ackermann

        # Update pose based on velocities
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vtheta * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.theta = math.fmod(self.theta, 2 * math.pi)  # Normalize angle

        self.last_joint_states_time = current_time
        # self.get_logger().debug(f'Odometry update: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}, vx={self.vx:.2f}, vtheta={self.vtheta:.2f}')

    def imu_callback(self, msg: Imu):
        # Extract yaw from IMU quaternion
        _, _, imu_yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        if self.last_imu_yaw is not None:
            # Fuse IMU yaw into current pose
            current_theta = self.theta
            # Calculate difference to fuse
            # Need to handle angle wrapping for fusion
            yaw_diff = imu_yaw - self.last_imu_yaw
            # Normalize yaw_diff to [-pi, pi]
            yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi

            # Simple fusion example:
            # The IMU provides absolute orientation, so we directly use its yaw
            # and weigh it. This assumes IMU yaw is reliable.
            # More sophisticated fusion (e.g., EKF) would be better.

            # If IMU is the primary source for orientation:
            # self.theta = imu_yaw

            # If fusing: update vtheta based on IMU angular velocity or use IMU's absolute orientation
            # For simplicity, we directly update theta if the IMU is reliable enough

            # Simple direct update of theta from IMU if IMU is heavily weighted for orientation
            # This is a simplification; a proper filter like an EKF would combine angular velocities.
            self.theta = (self.theta * (1.0 - self.imu_orientation_weight)) + (imu_yaw * self.imu_orientation_weight)
            self.theta = math.fmod(self.theta, 2 * math.pi)  # Normalize angle

        self.last_imu_yaw = imu_yaw
        # self.get_logger().debug(f'Received IMU: yaw={imu_yaw:.2f}')

    def publish_odom(self):
        current_time = self.get_clock().now()

        # Publish the transform (odom -> base_footprint)
        odom_tf = TransformStamped()
        odom_tf.header.stamp = current_time.to_msg()
        odom_tf.header.frame_id = self.odom_frame
        odom_tf.child_frame_id = self.footprint_frame
        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0  # Assuming 2D robot
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.theta)
        odom_tf.transform.rotation.x = qx
        odom_tf.transform.rotation.y = qy
        odom_tf.transform.rotation.z = qz
        odom_tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(odom_tf)

        # Publish the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.footprint_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta

        # Add covariance (simple diagonal for now, adjust based on sensor noise)
        odom_msg.pose.covariance = np.diag([
            0.05, 0.05, 0.0, 0.0, 0.0, 0.03  # x, y, z, roll, pitch, yaw
        ]).flatten().tolist()
        odom_msg.twist.covariance = np.diag([
            0.05, 0.05, 0.0, 0.0, 0.0, 0.03  # vx, vy, vz, vroll, vpitch, vyaw
        ]).flatten().tolist()

        self.odom_publisher.publish(odom_msg)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw) -> Tuple[float, float, float, float]:
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

    @staticmethod
    def quaternion_to_euler(x, y, z, w) -> Tuple[float, float, float]:
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()