import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleDrive(Node):
    def __init__(self):
        super().__init__('simple_drive')
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_timer(0.1, self.__publish_command)
        self.i=0

    def __publish_command(self):
        if self.i<400:
            command_message = Twist()
            command_message.linear.x = 0.1
            command_message.angular.z = 0.5
            self.__publisher.publish(command_message)
            self.get_logger().info('I: "%s"' % self.i)
        else:
            command_message = Twist()
            command_message.linear.x = 0.0
            command_message.angular.z = 0.0
            self.__publisher.publish(command_message)
        self.i=self.i+1


def main(args=None):
    rclpy.init(args=args)
    driver = SimpleDrive()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()