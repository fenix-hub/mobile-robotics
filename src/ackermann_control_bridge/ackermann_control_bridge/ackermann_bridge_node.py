import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class AckermannBridge(Node):
    def __init__(self):
        super().__init__('ackermann_bridge')
        
        # Publishers for group-joint controllers (they listen on <n>/commands)
        self.pub_rear_left_vel = self.create_publisher(
            Float64MultiArray,
            '/rear_left_wheel_velocity_controller/commands',
            10)
        self.pub_rear_right_vel = self.create_publisher(
            Float64MultiArray,
            '/rear_right_wheel_velocity_controller/commands',
            10)
        self.pub_fl_steer_pos = self.create_publisher(
            Float64MultiArray,
            '/front_left_steering_position_controller/commands',
            10)
        self.pub_fr_steer_pos = self.create_publisher(
            Float64MultiArray,
            '/front_right_steering_position_controller/commands',
            10)

        # Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Robot parameters (should match your URDF)
        self.wheelbase = 0.4      # [m] - distance between front and rear axles
        self.track_width = 0.3    # [m] - distance between left and right wheels
        self.wheel_radius = 0.05  # [m] - wheel radius

        # Friction compensation factor - helps with wheel slip
        self.friction_compensation = 1.5  # Increased from 1.2 to provide more torque

        # Turning balance factor - helps with uneven turning
        self.turning_balance = 0.9  # Slightly reduce turning effect to prevent over-rotation

        # Minimum linear velocity for forward motion to be considered significant
        self.min_linear_vel = 0.05

        self.get_logger().info('Ackermann Bridge Node Started with improved forward motion handling')

    def cmd_vel_callback(self, msg: Twist):
        linear_vel = msg.linear.x    # Forward velocity [m/s]
        angular_vel = msg.angular.z * self.turning_balance  # Angular velocity [rad/s] with balance factor

        # Log incoming command for debugging
        self.get_logger().info(f'Received cmd_vel: linear={linear_vel:.3f}, angular={angular_vel:.3f}')

        # Apply a small deadzone to angular velocity to prevent minor unwanted rotations
        if abs(angular_vel) < 0.05:
            angular_vel = 0.0

        # Handle straight line motion (no turning)
        if abs(angular_vel) < 1e-6:
            # Apply friction compensation for better straight-line motion
            wheel_angular_vel = (linear_vel / self.wheel_radius) * self.friction_compensation

            # For very small linear velocities, apply a minimum threshold
            if abs(linear_vel) < self.min_linear_vel and abs(linear_vel) > 1e-6:
                wheel_angular_vel = math.copysign(self.min_linear_vel / self.wheel_radius * self.friction_compensation, linear_vel)

            steering_angle_left = 0.0
            steering_angle_right = 0.0

            # Both rear wheels same speed
            rear_left_vel = wheel_angular_vel
            rear_right_vel = wheel_angular_vel

            self.get_logger().info(f'Straight motion: wheel_vel={wheel_angular_vel:.3f}')

        else:
            # Calculate turning radius from vehicle center
            if abs(linear_vel) < 1e-6:
                # Pure rotation - not typical for Ackermann but handle it
                steering_angle_left = math.copysign(0.785, angular_vel)  # Max steering ~45 degrees
                steering_angle_right = math.copysign(0.785, angular_vel)

                # For in-place rotation, use differential speeds with increased torque
                rotation_speed = 0.5 * self.wheel_radius * abs(angular_vel) * self.friction_compensation  # Increased from 0.2
                rear_left_vel = -math.copysign(rotation_speed, angular_vel)
                rear_right_vel = math.copysign(rotation_speed, angular_vel)

                self.get_logger().info(f'Pure rotation: rotation_speed={rotation_speed:.3f}')
            else:
                # Normal Ackermann steering calculation
                turn_radius = linear_vel / angular_vel

                # Calculate inner and outer wheel distances to instantaneous center of rotation (ICR)
                if angular_vel > 0:  # Left turn
                    inner_radius = turn_radius - self.track_width / 2
                    outer_radius = turn_radius + self.track_width / 2
                else:  # Right turn
                    inner_radius = turn_radius + self.track_width / 2
                    outer_radius = turn_radius - self.track_width / 2

                # Calculate wheel velocities based on their distance from ICR
                # Apply friction compensation for more effective motion
                inner_vel = (abs(angular_vel) * abs(inner_radius) / self.wheel_radius) * self.friction_compensation
                outer_vel = (abs(angular_vel) * abs(outer_radius) / self.wheel_radius) * self.friction_compensation

                # Assign velocities to left/right wheels based on turn direction
                if angular_vel > 0:  # Left turn
                    rear_left_vel = math.copysign(inner_vel, linear_vel)
                    rear_right_vel = math.copysign(outer_vel, linear_vel)
                else:  # Right turn
                    rear_left_vel = math.copysign(outer_vel, linear_vel)
                    rear_right_vel = math.copysign(inner_vel, linear_vel)

                # Front wheel steering angles (Ackermann geometry)
                L = self.wheelbase
                W = self.track_width
                r_abs = abs(turn_radius)

                if angular_vel > 0:  # Left turn
                    steering_angle_left = math.atan2(L, r_abs - W / 2)
                    steering_angle_right = math.atan2(L, r_abs + W / 2)
                else:  # Right turn
                    steering_angle_left = -math.atan2(L, r_abs + W / 2)
                    steering_angle_right = -math.atan2(L, r_abs - W / 2)

                self.get_logger().info(f'Turning: inner_vel={inner_vel:.3f}, outer_vel={outer_vel:.3f}')

        # Limit steering angles to physical constraints (-45° to +45°)
        max_steering = 0.785  # ~45 degrees in radians
        steering_angle_left = max(-max_steering, min(max_steering, steering_angle_left))
        steering_angle_right = max(-max_steering, min(max_steering, steering_angle_right))

        # Create messages
        rear_left_msg = Float64MultiArray(data=[rear_left_vel])
        rear_right_msg = Float64MultiArray(data=[rear_right_vel])
        front_left_msg = Float64MultiArray(data=[steering_angle_left])
        front_right_msg = Float64MultiArray(data=[steering_angle_right])

        # Publish commands
        self.pub_rear_left_vel.publish(rear_left_msg)
        self.pub_rear_right_vel.publish(rear_right_msg)
        self.pub_fl_steer_pos.publish(front_left_msg)
        self.pub_fr_steer_pos.publish(front_right_msg)

        # Debug logging
        self.get_logger().debug(f'Steering: L={steering_angle_left:.3f}, R={steering_angle_right:.3f}')
        self.get_logger().debug(f'Wheel vel: L={rear_left_vel:.3f}, R={rear_right_vel:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = AckermannBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
