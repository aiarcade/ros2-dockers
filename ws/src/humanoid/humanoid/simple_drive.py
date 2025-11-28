import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleDrive(Node):
    def __init__(self):
        super().__init__('simple_drive')
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_timer(2.0, self.__publish_command)
        self.arms_up = False

    def __publish_command(self):
        command_message = Twist()
        
        # Toggle arms up/down every 2 seconds
        if self.arms_up:
            command_message.linear.x = 0.0  # Arms down
            self.get_logger().info('Arms down')
        else:
            command_message.linear.x = 1.0  # Arms up
            self.get_logger().info('Arms up')
        
        self.arms_up = not self.arms_up
        command_message.angular.z = 0.0
        self.__publisher.publish(command_message)

def main(args=None):
    rclpy.init(args=args)
    driver = SimpleDrive()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()