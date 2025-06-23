#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10
        )
        # Wait a moment to ensure publisher is ready
        time.sleep(2.0)
        self.publish_initial_pose()
        self.get_logger().info("Published initial pose to AMCL")

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set the position (x, y, z)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # Set orientation - 0 yaw (identity quaternion)
        msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        
        # Set covariance (36-element array)
        covariance = [0.0] * 36
        covariance[0] = 0.25  # x variance
        covariance[7] = 0.25  # y variance
        covariance[35] = 0.0685  # yaw variance
        msg.pose.covariance = covariance
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    
    # Shutdown after publishing
    rclpy.shutdown()

if __name__ == '__main__':
    main()
