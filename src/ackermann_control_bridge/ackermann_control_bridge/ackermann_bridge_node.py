import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class AckermannBridge(Node):
    def __init__(self):
        super().__init__('ackermann_bridge')
        
        # Publishers for group-joint controllers (they listen on <name>/commands)
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

        self.get_logger().info('Ackermann Bridge Node Started')

    def cmd_vel_callback(self, msg: Twist):
        linear_vel = msg.linear.x    # Forward velocity [m/s]
        angular_vel = msg.angular.z  # Angular velocity [rad/s]

        # Handle straight line motion (no turning)
        if abs(angular_vel) < 1e-6:
            # Straight line motion
            wheel_angular_vel = linear_vel / self.wheel_radius
            steering_angle_left = 0.0
            steering_angle_right = 0.0
            
            # Both rear wheels same speed
            rear_left_vel = wheel_angular_vel
            rear_right_vel = wheel_angular_vel
            
        else:
            # Calculate turning radius from vehicle center
            if abs(linear_vel) < 1e-6:
                # Pure rotation - not typical for Ackermann but handle it
                steering_angle_left = math.copysign(0.785, angular_vel)  # Max steering ~45 degrees
                steering_angle_right = math.copysign(0.785, angular_vel)
                rear_left_vel = 0.0
                rear_right_vel = 0.0
            else:
                # Normal Ackermann steering calculation
                turn_radius = linear_vel / angular_vel  # Radius from vehicle center
                
                # Calculate individual wheel speeds and steering angles
                # For rear wheels (differential drive on rear axle)
                if turn_radius > 0:  # Left turn
                    # Inner wheel (left) slower, outer wheel (right) faster
                    left_turn_radius = turn_radius - self.track_width / 2
                    right_turn_radius = turn_radius + self.track_width / 2
                else:  # Right turn
                    # Inner wheel (right) slower, outer wheel (left) faster
                    left_turn_radius = turn_radius + self.track_width / 2
                    right_turn_radius = turn_radius - self.track_width / 2
                
                # Calculate rear wheel angular velocities
                rear_left_vel = (angular_vel * left_turn_radius) / self.wheel_radius
                rear_right_vel = (angular_vel * right_turn_radius) / self.wheel_radius
                
                # Calculate front wheel steering angles (Ackermann geometry)
                L = self.wheelbase
                W = self.track_width
                
                if turn_radius > 0:  # Left turn
                    # Left wheel (inner) has larger steering angle
                    steering_angle_left = math.atan2(L, abs(turn_radius) - W/2)
                    steering_angle_right = math.atan2(L, abs(turn_radius) + W/2)
                    # Preserve sign for left turn
                    steering_angle_left = math.copysign(steering_angle_left, angular_vel)
                    steering_angle_right = math.copysign(steering_angle_right, angular_vel)
                else:  # Right turn  
                    # Right wheel (inner) has larger steering angle
                    steering_angle_left = math.atan2(L, abs(turn_radius) + W/2)
                    steering_angle_right = math.atan2(L, abs(turn_radius) - W/2)
                    # Preserve sign for right turn
                    steering_angle_left = math.copysign(steering_angle_left, angular_vel)
                    steering_angle_right = math.copysign(steering_angle_right, angular_vel)

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
        self.get_logger().debug(
            f'cmd_vel: lin={linear_vel:.2f}, ang={angular_vel:.2f} | '
            f'Steering: L={math.degrees(steering_angle_left):.1f}°, R={math.degrees(steering_angle_right):.1f}° | '
            f'Rear wheels: L={rear_left_vel:.2f}, R={rear_right_vel:.2f} rad/s'
        )

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
