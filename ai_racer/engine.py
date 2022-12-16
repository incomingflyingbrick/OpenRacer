from jetcam.csi_camera import CSICamera
import tensorflow_hub as hub
import tensorflow as tf

from multiprocessing.spawn import prepare
import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import String
from time import *
from sensor_msgs.msg import Joy
import subprocess
import uuid
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
import PIL
from sensor_msgs.msg import JointState, Image
from rclpy.qos import QoSProfile
import tensorflow_hub as hub
import traceback
import numpy as np
import cv2
from std_msgs.msg import Int32MultiArray

kit = ServoKit(channels=16)
kit.servo[0].angle = 72

cv2.startWindowThread()


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('engine_node')
        # self._tf_publisher = StaticTransformBroadcaster(self)
        self.is_free_drive = False
        self.is_race = True
        self.isCollecting = False
        self.is_detection_mode = False
        self.is_start_race = False
        # qos_profile = QoSProfile(depth=10)
        # self.joint_pub = self.create_publisher(JointState, 'joint_states',qos_profile)
        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.prepareDataCollection()
        self.steer = 0.0
        self.throttle = 0.0
        self.frame_counter = 0
        self.turn_value = 0.0
        self.throttle_value = 0.0
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_CallBack,
            10)

        self.subscription
        self.get_logger().info('Engine Node: start inference engine')
        if self.isCollecting == True:
            self.get_logger().info("COLLECTION mode is on")
        if self.is_detection_mode == True:
            self.get_logger().info("DETECTION mode is on")
            self.get_logger().info('model loading......')
            self.model = hub.load("/home/jetson/Downloads/ssd_mobilenet_v2")
            self.get_logger().info('model success load!')
        if self.is_race == True:
            self.get_logger().info("RACE mode is on")
            self.get_logger().info('model loading......')
            self.race_model = tf.keras.models.load_model(
                '/home/jetson/Downloads/model/')
            self.get_logger().info('model success load!')
        self.get_logger().info('Engine Node: init sequence end')
        self.camera = CSICamera(
            width=328, height=246, capture_width=328, capture_height=246, capture_fps=30)
        self.camera.running = True
        self.camera.observe(self.cameraCallback, names='value')
        self.get_logger().info('Camera init sequence end')

    def cameraCallback(self, change):
        
        if self.isCollecting:
            if self.throttle_value != 0.0:
                self.saveData(self.turn_value, change)
        if self.is_detection_mode:
            self.frame_counter+=1
            self.inference(change)
        if self.is_race:
            # image = change['new']
            # cv2.putText(img=image, text='Steer: '+str(self.steer), org=(
            # 16, 200), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.45, color=(255, 255, 255), thickness=1)
            # cv2.putText(img=image, text='Throttle: '+str(self.throttle), org=(
            # 16, 220), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.45, color=(255, 255, 255), thickness=1)
            # cv2.namedWindow("camera_view")
            # cv2.imshow('camera_view', image)
            self.frame_counter+=1
            self.race_inference(change)
        if self.is_free_drive:
            cv2.namedWindow("camera_view")
            cv2.imshow('camera_view', change['new'])
        

    def race_inference(self, change):
        #self.get_logger().info('race inference called')
        if self.is_start_race == True and self.frame_counter>=15:
            #self.get_logger().info('race inference starting')
            
            try:
                result = self.race_model.predict(
                    np.asarray([change['new']]), batch_size=1)
                self.steer = result[0][0]
                self.get_logger().info(
                    "Turn: "+str(self.steer))
                #self.get_logger().info(result)
                #steer
                if -1.0<=self.steer<=1.0:
                    turn = self.steer*-1.0
                    y = turn/(1.0/35.0)
                    y = 72+y
                    kit.servo[0].angle = int(y)
                #throttle
                #kit.servo[1].angle = 97
                # if -1.0<=self.throttle<=1.0:
                #     the = self.throttle
                #     t = the/(1.0/6.0)
                #     if the == 0.0 or the == -0.0:
                #         kit.servo[1].angle = 90
                #     else:
                #         if the > 0:
                #             kit.servo[1].angle = 90+t
                #         else:
                #             kit.servo[1].angle = 90+t-8
                # cv2.namedWindow("camera_view")
                # image = change['new']
                # cv2.putText(img=image, text='Steer: '+str(self.steer), org=(
                #     16, 200), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.45, color=(255, 255, 255), thickness=1)
                # cv2.putText(img=image, text='Throttle: '+str(self.throttle), org=(
                #     16, 220), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.45, color=(255, 255, 255), thickness=1)
                # cv2.imshow('camera_view', image)
            except Exception as e:
                self.get_logger().info(traceback.format_exc())
            self.frame_counter = 0

    def inference(self, change):
        if self.frame_counter<8:
            return

        tolerance = 40
        tolerance_hight = 60
        image = change['new']
        try:
            batch = [image]
            data = np.asarray(batch)
            start_time = time()
            result = self.model(tf.convert_to_tensor(data))
            self.get_logger().info("inference time: "+str((time()-start_time)*1000)+'ms')
            box_tensor = result['detection_boxes']
            class_tensor = result['detection_classes']
            # print('class:')
            # print(class_tensor)
            detected_index = tf.where(tf.equal(1.0, class_tensor[0]))
            score_index = detected_index[0][0]
            score_tensor = result['detection_scores']
            # print('score shape:')
            if score_tensor[0][score_index] >= 0.5:
                # print(draw boxes)
                box_data = box_tensor[0]
                i = box_data[score_index]
                centre_x = int(
                    (int(i[3]*image.shape[1])-int(i[1]*image.shape[1]))/2) + int(i[1]*image.shape[1])
                centre_y = int(
                    (int(i[2]*image.shape[0])-int(i[0]*image.shape[0]))/2) + int(i[0]*image.shape[0])
                cv2.line(image,(centre_x,centre_y),(int(image.shape[1]/2),int(image.shape[0]/2)),(0, 0,200), 2)

                cv2.rectangle(image, (int(i[1]*image.shape[1]), int(i[0]*image.shape[0])),
                            (int(i[3]*image.shape[1]), int(i[2]*image.shape[0])), (255, 0, 0), 2)
                cv2.namedWindow("camera_view")
                cv2.imshow('camera_view', image)
                target_centre_x = int(image.shape[1]/2)
                target_centre_y = int(image.shape[0]/2)
                if centre_x - target_centre_x > 0 and abs(centre_x - target_centre_x) > tolerance:
                    self.get_logger().info('RIGHT==============>')
                    kit.servo[0].angle = int(72+10)
                elif centre_x - target_centre_x < 0 and abs(centre_x - target_centre_x) > tolerance:
                    self.get_logger().info('<==============LEFT')
                    kit.servo[0].angle = int(72-10)
                else:
                    self.get_logger().info('<==============CENTRE===========')
                    kit.servo[0].angle = int(72)

                if centre_y - target_centre_y > 0 and abs(centre_y - target_centre_y) > tolerance_hight:
                    self.get_logger().info('==========REVERSE==========')
                    kit.servo[1].angle = 90-10
                elif centre_y - target_centre_y < 0 and abs(centre_y - target_centre_y) > tolerance_hight:
                    self.get_logger().info(
                        '========================================FORWARD========================================')
                    kit.servo[1].angle = 96
                else:
                    self.get_logger().info('==================STAY=================')
                    kit.servo[1].angle = 90

        except Exception as e:
            self.get_logger().info(traceback.format_exc())
            # logging.error()
        self.frame_counter = 0

    def saveData(self, steering, image_data):
        if self.isCollecting:
            file_path = 'snapshots/' + \
                str(uuid.uuid1()) + '_'+str(round(steering, 7)) + '_.png'
            im = PIL.Image.fromarray(image_data['new'])
            im.save(file_path)
            self.get_logger().info('saved data:'+file_path)

    def joy_CallBack(self, msg):
        # self.get_logger().info('Button:'+str(msg.buttons))
        # self.get_logger().info('Axes:'+str(msg.axes))
        self.turn_value = msg.axes[2]
        self.throttle_value = msg.axes[1]

        if msg.buttons[0] == 1:#button A
            self.prepareEsc()
        if msg.buttons[3] == 1 and self.is_race == True:
            self.is_start_race = True
            self.get_logger().info('X pressed')
        if msg.buttons[2] == 1 and self.is_race == True:
            self.is_start_race = False
            self.get_logger().info('Y pressed')
        #turn
        if self.isCollecting == True or self.is_free_drive == True:
            turn = msg.axes[2]*-1.0
            y = turn/(1.0/40.0)
            y = 72+y
            kit.servo[0].angle = int(y)
        # throttle
        if self.is_free_drive == True or self.isCollecting == True or self.is_race == True:
            the = msg.axes[1]
            t = the/(1.0/6.5)
            if the == 0.0 or the == -0.0:
                kit.servo[1].angle = 90
            else:
                if the > 0:
                    kit.servo[1].angle = 90+t
                else:
                    kit.servo[1].angle = 90+t-8

        # controll 3D model
        # x_model = msg.axes[2]
        # y_model = msg.axes[1]
        # self.make_transforms(x_model,y_model)

    def make_transforms(self, x, y):
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
        # self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(static_transformStamped)

    def prepareEsc(self):
        self.get_logger().info("ESC cal start")
        for i in range(50):
            kit.servo[1].angle = 90
            sleep(0.1)
        self.get_logger().info("ESC cal end")

    def prepareDataCollection(self):
        subprocess.call(['rm', '-r', '-f', 'snapshots'])
        subprocess.call(['mkdir', '-p', 'snapshots'])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
