
from multiprocessing.spawn import prepare
import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import String
from time import *
from sensor_msgs.msg import Joy
import subprocess
from jetcam.csi_camera import CSICamera
import traitlets
import uuid
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from PIL import Image
from sensor_msgs.msg import JointState,Image
from rclpy.qos import QoSProfile
import tensorflow_hub as hub

model = hub.load("https://hub.tensorflow.google.cn/tensorflow/ssd_mobilenet_v2/2")

kit = ServoKit(channels=16)
kit.servo[0].angle = 72



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # self._tf_publisher = StaticTransformBroadcaster(self)
        
        self.isCollecting = False
        # qos_profile = QoSProfile(depth=10)
        # self.joint_pub = self.create_publisher(JointState, 'joint_states',qos_profile)
        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.prepareDataCollection()
        self.camera = CSICamera(width=328, height=246,capture_fps=10)
        self.camera.running=True
        self.camera.observe(self.cameraCallback,names='value')
        self.turn_value = 0.0
        self.throttle_value = 0.0

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_CallBack,
            10)

        self.subscription
        self.get_logger().info('Engine Node init success! Ready for joy stick input!')

    def cameraCallback(self,change):
        #self.get_logger().info(str(change['new']))
        self.inference(change['new'])
        # if self.turn_value!=0.0 or self.throttle_value!=0.0:
        #     self.saveData(self.turn_value,self.throttle_value,change['new'])
    
    def inference(self,image):
        self.get_logger().info("inference")
        pass

    def saveData(self,steering,throttle,image_data):
        if self.isCollecting:
            file_path = 'snapshots/' + str(uuid.uuid1()) +'_'+str(round(steering,5))+'_'+str(round(throttle,5))+ '_.png'
            im = Image.fromarray(image_data)
            im.save(file_path)
            self.get_logger().info('saved data:'+file_path)

    def joy_CallBack(self,msg):
        # self.get_logger().info('Button:'+str(msg.buttons))
        # self.get_logger().info('Axes:'+str(msg.axes))
        self.turn_value = msg.axes[2]
        self.throttle_value = msg.axes[1]

        if msg.buttons[0]==1:
            self.prepareEsc()
        #turn
        turn = msg.axes[2]*-1.0
        y=turn/(1.0/40.0)
        y=72+y
        kit.servo[0].angle = int(y)
        #throttle
        the = msg.axes[1]
        t= the/(1.0/9.0)
        if the==0.0 or the==-0.0:
            kit.servo[1].angle = 90
        else:
            if the > 0:
                kit.servo[1].angle = 90+t
            else:
                kit.servo[1].angle = 90+t-8

        #controll 3D model
        # x_model = msg.axes[2]
        # y_model = msg.axes[1]
        # self.make_transforms(x_model,y_model)


    def make_transforms(self, x,y):
      static_transformStamped = TransformStamped()
      static_transformStamped.header.stamp = self.get_clock().now().to_msg()
      static_transformStamped.header.frame_id = 'world'
      static_transformStamped.child_frame_id = 'base_link'
      static_transformStamped.transform.translation.x = x
      static_transformStamped.transform.translation.y = y
      static_transformStamped.transform.translation.z = 0.0
      joint_state = JointState()
      now = self.get_clock().now()
      joint_state.header.stamp = now.to_msg()
      joint_state.name = ['base_to_sensor_joint']
      #self.joint_pub.publish(joint_state)
      self.broadcaster.sendTransform(static_transformStamped)

      
    
    def prepareEsc(self):
        self.get_logger().info("电调自检验开始")
        for i in range(50):
            kit.servo[1].angle = 90
            sleep(0.1)
        self.get_logger().info("电调自检验完成")

    def prepareDataCollection(self):
        subprocess.call(['rm', '-r','-f', 'snapshots'])
        subprocess.call(['mkdir', '-p', 'snapshots'])
    


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
