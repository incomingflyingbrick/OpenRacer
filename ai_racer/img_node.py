import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('img_node')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'camera_image_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('img_node is up and runnning')

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()