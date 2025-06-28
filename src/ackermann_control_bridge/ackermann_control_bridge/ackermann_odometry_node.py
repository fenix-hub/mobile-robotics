#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class AckermannOdometryNode(Node):
    def __init__(self):
        super().__init__('ackermann_odometry_node')

        # ——— parameters —————————————————————————————————————————————————
        self.declare_parameter('wheelbase',                  0.4)
        self.declare_parameter('track_width',                0.3)
        self.declare_parameter('wheel_radius',               0.05)
        self.declare_parameter('odom_frame',                 'odom')
        self.declare_parameter('base_frame',                 'base_link')
        self.declare_parameter('footprint_frame',            'base_footprint')
        self.declare_parameter('publish_rate',               50.0)

        # IMU fusion
        self.declare_parameter('use_imu',                    True)
        self.declare_parameter('imu_weight',                 0.6)

        # encoder / cmd_vel fusion
        self.declare_parameter('wheel_encoder_weight',       0.8)
        self.declare_parameter('cmd_vel_weight',             0.3)
        self.declare_parameter('velocity_timeout',           0.5)

        # filtering
        self.declare_parameter('max_linear_accel',           2.0)
        self.declare_parameter('max_angular_accel',          4.0)

        # deadband thresholds
        self.declare_parameter('linear_deadband',            0.001)
        self.declare_parameter('angular_deadband',           0.01)

        # read parameters
        self.wheelbase            = self.get_parameter('wheelbase').value
        self.track_width          = self.get_parameter('track_width').value
        self.wheel_radius         = self.get_parameter('wheel_radius').value
        self.odom_frame           = self.get_parameter('odom_frame').value
        self.base_frame           = self.get_parameter('base_frame').value
        self.footprint_frame      = self.get_parameter('footprint_frame').value
        self.publish_rate         = self.get_parameter('publish_rate').value

        self.use_imu              = self.get_parameter('use_imu').value
        self.imu_weight           = self.get_parameter('imu_weight').value

        self.wheel_encoder_weight = self.get_parameter('wheel_encoder_weight').value
        self.cmd_vel_weight       = self.get_parameter('cmd_vel_weight').value
        self.velocity_timeout     = self.get_parameter('velocity_timeout').value

        self.max_linear_accel     = self.get_parameter('max_linear_accel').value
        self.max_angular_accel    = self.get_parameter('max_angular_accel').value

        self.linear_deadband      = self.get_parameter('linear_deadband').value
        self.angular_deadband     = self.get_parameter('angular_deadband').value

        # ——— odometry state ————————————————————————————————————————————————
        self.x    = 0.0
        self.y    = 0.0
        self.theta= 0.0
        self.vx   = 0.0
        self.vtheta = 0.0

        # last inputs
        self.last_time           = self.get_clock().now()
        self.last_joint_time: Optional[rclpy.time.Time] = None
        self.last_cmd_time: Optional[rclpy.time.Time]   = None

        self.last_cmd_linear_x   = 0.0
        self.last_cmd_angular_z  = 0.0

        self.last_wheel_velocities = {
            'front_left': 0.0,
            'front_right':0.0,
            'rear_left':  0.0,
            'rear_right': 0.0
        }
        self.last_vx             = 0.0
        self.last_vtheta         = 0.0

        # IMU storage
        self.last_imu_time        = None
        self.imu_angular_velocity = 0.0
        self.imu_yaw              = 0.0

        # ——— publishers, subscribers, broadcaster ————————————————————
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Twist,      '/cmd_vel',     self.cmd_vel_callback,     qos_sensor)
        self.create_subscription(JointState, '/joint_states',self.joint_state_callback,qos_sensor)
        self.create_subscription(Imu,        '/imu/data',    self.imu_callback,         qos_sensor)

        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_publisher = self.create_publisher(Odometry, '/odom', odom_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # timer
        self.create_timer(1.0 / self.publish_rate, self.publish_odom)

        self.get_logger().info('Ackermann Odometry Node started.')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_linear_x  = msg.linear.x
        self.last_cmd_angular_z = msg.angular.z
        self.last_cmd_time      = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now()
        self.last_imu_time = now
        # yaw from quaternion
        q = msg.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.imu_yaw              = math.atan2(siny, cosy)
        self.imu_angular_velocity = msg.angular_velocity.z

    def joint_state_callback(self, msg: JointState):
        now = self.get_clock().now()
        if self.last_joint_time is None:
            dt = (now - self.last_time).nanoseconds / 1e9
        else:
            dt = (now - self.last_joint_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # extract wheel velocities and steering
        fl=fr=rl=rr=0.0
        steer_sum=0.0; steer_cnt=0
        for i,name in enumerate(msg.name):
            if name=='front_left_wheel_joint':  fl = msg.velocity[i]
            if name=='front_right_wheel_joint': fr = msg.velocity[i]
            if name=='rear_left_wheel_joint':   rl = msg.velocity[i]
            if name=='rear_right_wheel_joint':  rr = msg.velocity[i]
            if name=='front_left_steering_joint':
                steer_sum += msg.position[i]; steer_cnt+=1
            if name=='front_right_steering_joint':
                steer_sum += msg.position[i]; steer_cnt+=1

        steering = steer_sum/steer_cnt if steer_cnt>0 else 0.0
        # convert to m/s
        rl_mps = rl * self.wheel_radius
        rr_mps = rr * self.wheel_radius

        # deadband on raw velocities
        if abs(rl_mps) < self.linear_deadband: rl_mps = 0.0
        if abs(rr_mps) < self.linear_deadband: rr_mps = 0.0

        linear_x_enc = (rl_mps + rr_mps)/2.0

        # angular from steering
        if abs(linear_x_enc)>self.linear_deadband and abs(steering)>1e-3:
            angular_z_enc = linear_x_enc * math.tan(steering) / self.wheelbase
        else:
            # fallback diff-wheel
            angular_z_enc = (rr_mps - rl_mps)/self.track_width

        # deadband angular
        if abs(angular_z_enc) < self.angular_deadband:
            angular_z_enc = 0.0

        # accel limiting
        lin_acc = (linear_x_enc - self.last_vx)/dt
        ang_acc = (angular_z_enc - self.last_vtheta)/dt
        if abs(lin_acc) > self.max_linear_accel:
            linear_x_enc = self.last_vx + math.copysign(self.max_linear_accel*dt, lin_acc)
        if abs(ang_acc) > self.max_angular_accel:
            angular_z_enc = self.last_vtheta + math.copysign(self.max_angular_accel*dt, ang_acc)

        # fuse with cmd_vel
        total_w_lin = self.wheel_encoder_weight
        total_w_ang = self.wheel_encoder_weight
        fused_lin = linear_x_enc * self.wheel_encoder_weight
        fused_ang = angular_z_enc * self.wheel_encoder_weight

        if self.last_cmd_time is not None:
            age = (now - self.last_cmd_time).nanoseconds / 1e9
            if age < self.velocity_timeout:
                fused_lin += self.last_cmd_linear_x * self.cmd_vel_weight
                fused_ang += self.last_cmd_angular_z * self.cmd_vel_weight
                total_w_lin += self.cmd_vel_weight
                total_w_ang += self.cmd_vel_weight

        fused_lin /= total_w_lin
        fused_ang /= total_w_ang

        self.vx     = fused_lin
        self.vtheta = fused_ang

        # fuse with IMU for orientation
        if self.use_imu and self.last_imu_time:
            imu_age = (now - self.last_imu_time).nanoseconds / 1e9
            if imu_age < 0.1:
                # trust IMU for yaw
                self.theta = self.imu_yaw
                # fuse angular rate too
                fused_ang = (
                    angular_z_enc*self.wheel_encoder_weight +
                    self.imu_angular_velocity*self.imu_weight
                )/(self.wheel_encoder_weight + self.imu_weight)
                self.vtheta = fused_ang

        # integrate position
        dx = self.vx * math.cos(self.theta) * dt
        dy = self.vx * math.sin(self.theta) * dt
        self.x += dx
        self.y += dy
        # theta set by IMU fusion above

        # store for next iter
        self.last_vx           = self.vx
        self.last_vtheta       = self.vtheta
        self.last_joint_time   = now

    def publish_odom(self):
        now = self.get_clock().now()

        # TF: odom → base_footprint
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.footprint_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.x, \
        tf.transform.rotation.y, \
        tf.transform.rotation.z, \
        tf.transform.rotation.w = self.euler_to_quaternion(0,0,self.theta)
        self.tf_broadcaster.sendTransform(tf)

        # Odometry msg
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.footprint_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x, \
        odom.pose.pose.orientation.y, \
        odom.pose.pose.orientation.z, \
        odom.pose.pose.orientation.w = self.euler_to_quaternion(0,0,self.theta)

        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.angular.z = self.vtheta

        # you can keep your covariance settings here…

        self.odom_publisher.publish(odom)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw) -> Tuple[float,float,float,float]:
        cr = math.cos(roll/2); sr = math.sin(roll/2)
        cp = math.cos(pitch/2); sp = math.sin(pitch/2)
        cy = math.cos(yaw/2); sy = math.sin(yaw/2)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        return qx, qy, qz, qw


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
