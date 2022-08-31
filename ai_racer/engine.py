
from multiprocessing.spawn import prepare
import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import String
from time import *
from sensor_msgs.msg import Joy

kit = ServoKit(channels=16)
kit.servo[0].angle = 72



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # self.subscription = self.create_subscription(
        #     String,
        #     'topic',
        #     self.listener_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_CallBack,
            10)

        self.subscription

    def joy_CallBack(self,msg):
        self.get_logger().info('Button:'+str(msg.buttons))
        self.get_logger().info('Axes:'+str(msg.axes))
        if msg.buttons[0]==1:
            self.prepareEsc()
        #turn
        turn = msg.axes[2]*-1.0
        y=turn/(1.0/25.0)
        y=72+y
        kit.servo[0].angle = int(y)
        #throttle
        the = msg.axes[1]
        t= the/(1.0/10.0)
        if the==0.0 or the==-0.0:
            kit.servo[1].angle = 90
        else:
            if the > 0:
                kit.servo[1].angle = 90+t
            else:
                kit.servo[1].angle = 90+t-7

    
    def prepareEsc(self):
        self.get_logger().info("电调自检验开始")
        for i in range(50):
            kit.servo[1].angle = 90
            sleep(0.1)
        self.get_logger().info("电调自检验完成")


    def listener_callback(self, msg):
        pass
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == 'w':
            kit.servo[1].angle = 100
        elif msg.data == 's':
            kit.servo[1].angle = 69
        elif msg.data == ' ':
            kit.servo[1].angle = 90


        if msg.data == 'a':
            kit.servo[0].angle = 57
        elif msg.data == 'd':
            kit.servo[0].angle = 87
        else:
            kit.servo[0].angle = 72

        
        if msg.data == 'k':
            self.prepareEsc()
    


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
