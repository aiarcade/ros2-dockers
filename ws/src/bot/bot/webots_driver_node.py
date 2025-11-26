#!/usr/bin/env python3
"""Standalone ROS2 node that runs the Webots driver"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class WebotsDriverNode(Node):
    def __init__(self):
        super().__init__('webots_driver_node')
        self.get_logger().info('Webots driver node started - waiting for Webots connection')
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        
    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = WebotsDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
