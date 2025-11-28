import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.__left_sensor_value = MAX_RANGE
        self.__right_sensor_value = MAX_RANGE

        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)
        
        self.create_timer(0.1, self.__publish_command)

    def __left_sensor_callback(self, message):
        # Treat 0 as invalid reading (no obstacle detected)
        if message.range > 0.001:
            self.__left_sensor_value = message.range
        else:
            self.__left_sensor_value = MAX_RANGE

    def __right_sensor_callback(self, message):
        # Treat 0 as invalid reading (no obstacle detected)
        if message.range > 0.001:
            self.__right_sensor_value = message.range
        else:
            self.__right_sensor_value = MAX_RANGE

    def __publish_command(self):
        command_message = Twist()
        
        # Check if obstacles are detected
        left_obstacle = self.__left_sensor_value < 0.9 * MAX_RANGE
        right_obstacle = self.__right_sensor_value < 0.9 * MAX_RANGE
        
        if left_obstacle or right_obstacle:
            # Slow down but keep some forward movement while turning
            command_message.linear.x = 0.05
            
            # Turn away from the obstacle
           
        self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()