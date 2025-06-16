# save this as odom_recorder.py in a ROS 2 Python package
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os

class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odom_recorder')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.cb, 10)
        self.file = open(os.path.expanduser('~/odom_path.csv'), 'w', newline='')
        self.csv = csv.writer(self.file)
        # header
        self.csv.writerow(['time', 'x', 'y', 'yaw'])

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # simple yaw extraction
        import math
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        self.csv.writerow([t, x, y, yaw])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

