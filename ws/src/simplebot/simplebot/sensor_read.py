import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist





class SimpleDrive(Node):
    def __init__(self):
        super().__init__('simple_drive')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)
    
    def __left_sensor_callback(self, message):
        self.__left_sensor_value = message.range
        print(self.__left_sensor_value)

    def __right_sensor_callback(self, message):
        self.__right_sensor_value = message.range
        print(self.__right_sensor_value)


    def timer_callback(self):
        
        command_message = Twist()

        command_message.linear.x = 0.1

        command_message.angular.z = -2.0

        self.__publisher.publish(command_message)

    def stop(self):
        command_message = Twist()
        self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    driver = SimpleDrive()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()