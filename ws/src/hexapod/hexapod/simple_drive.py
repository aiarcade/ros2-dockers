import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SimpleDrive(Node):
    def __init__(self):
        super().__init__('simple_drive')
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.__sensor_sub = self.create_subscription(
            String, 
            '/hexapod/sensors', 
            self.__sensor_callback, 
            10
        )
        self.create_timer(0.1, self.__publish_command)
        self.get_logger().info("SimpleDrive started - sending forward commands and listening to sensors")

    def __sensor_callback(self, msg):
        self.get_logger().info(f"Sensors: {msg.data}")

    def __publish_command(self):
        command_message = Twist()
        command_message.linear.x = 1.0
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