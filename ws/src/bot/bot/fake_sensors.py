import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__('fake_sensor_publisher')
        self.left_pub = self.create_publisher(Range, 'left_sensor', 1)
        self.right_pub = self.create_publisher(Range, 'right_sensor', 1)
        self.create_timer(0.1, self.publish_sensors)

    def publish_sensors(self):
        msg = Range()
        msg.range = 0.15
        msg.min_range = 0.0
        msg.max_range = 0.15
        self.left_pub.publish(msg)
        self.right_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
