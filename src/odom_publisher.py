#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.1  # Linear velocity
        self.vth = 0.1  # Angular velocity

    def timer_callback(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Update position
        self.x += self.vx * 0.1
        self.y += self.vx * 0.1

        # Set the Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Set the Twist (linear and angular velocity)
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.z = self.vth

        self.publisher_.publish(odom_msg)
        self.get_logger().info(f'Publishing: x: {self.x}, y: {self.y}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
