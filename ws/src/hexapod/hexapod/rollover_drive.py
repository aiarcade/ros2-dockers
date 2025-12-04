import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RolloverDrive(Node):
    def __init__(self):
        super().__init__('rollover_drive')
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.__sensor_sub = self.create_subscription(
            String, 
            '/hexapod/sensors', 
            self.__sensor_callback, 
            10
        )
        self.create_timer(0.1, self.__publish_command)
        self.state = 0  # 0: spin fast, 1: forward fast, 2: spin opposite
        self.counter = 0
        self.get_logger().info("RolloverDrive started - maximum speed rollover motion!")

    def __sensor_callback(self, msg):
        self.get_logger().info(f"Sensors: {msg.data}")

    def __publish_command(self):
        command_message = Twist()
        
        # Cycle through states every 20 steps (2 seconds)
        if self.counter >= 20:
            self.state = (self.state + 1) % 3
            self.counter = 0
            self.get_logger().info(f"State: {['SPIN_LEFT', 'FORWARD_FAST', 'SPIN_RIGHT'][self.state]}")
        
        if self.state == 0:
            # Spin fast left
            command_message.linear.x = 0.5
            command_message.angular.z = 2.0
        elif self.state == 1:
            # Forward at maximum speed
            command_message.linear.x = 3.0
            command_message.angular.z = 0.0
        else:  # state == 2
            # Spin fast right
            command_message.linear.x = 0.5
            command_message.angular.z = -2.0
        
        self.counter += 1
        self.__publisher.publish(command_message)

def main(args=None):
    rclpy.init(args=args)
    driver = RolloverDrive()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
