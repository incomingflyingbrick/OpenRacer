from jetcam.csi_camera import CSICamera
import tensorflow_hub as hub
import tensorflow as tf
import cv2
import numpy as np
import traceback
import logging
import queue

img_que = queue.Queue()

def cameraCallback(change):
    # self.get_logger().info(str(change['new']))
    image = change['new']
    try:
        batch = [image]
        data = np.asarray(batch)
        result = model(tf.convert_to_tensor(data))
        box_tensor = result['detection_boxes']
        class_tensor = result['detection_classes']
        print('class:')
        print(class_tensor)
        detected_index = tf.where(tf.equal(1.0,class_tensor[0]))
        score_index = detected_index[0][0]
        score_tensor = result['detection_scores']
        print('score shape:')
        if score_tensor[0][score_index] >=0.6:
            #print(draw boxes)
            box_data = box_tensor[0]
            i = box_data[score_index]
            centre_x =  int((int(i[3]*image.shape[1])-int(i[1]*image.shape[1]))/2) +int(i[1]*image.shape[1])
            centre_y =  int((int(i[2]*image.shape[0])-int(i[0]*image.shape[0]))/2) +int(i[0]*image.shape[0])
            cv2.line(image,(centre_x,centre_y),(int(image.shape[1]/2),int(image.shape[0]/2)),(50, 134, 76), 2)

            cv2.rectangle(image, (int(i[1]*image.shape[1]), int(i[0]*image.shape[0])),
                        (int(i[3]*image.shape[1]), int(i[2]*image.shape[0])), (255, 134, 76), 3)
            target_centre_x = int(image.shape[1]/2)
            target_centre_y = int(image.shape[0]/2)
            if centre_x - target_centre_x >0:
                print('turn right')
            else:
                print('turn left')

            if centre_y - target_centre_y > 0:
                print('reverse')
            else:
                print('forward')
            
            

    except Exception as e:
        logging.error(traceback.format_exc())
    print("inference done")
    #cv2.imshow("Result", image)
    cv2.rectangle(image, (int(image.shape[1]/2)-20, int(image.shape[0]/2)-20),
                     (int(image.shape[1]/2)+20, int(image.shape[0]/2)+20), (0, 200, 0), 1)
    img_que.put(image)




print('model loading')
model = hub.load(
    "https://hub.tensorflow.google.cn/tensorflow/ssd_mobilenet_v2/2")
print('model success load')

# frameWidth = 328
# frameHeight = 246
# cap = cv2.VideoCapture(0)
# cap.set(3, frameWidth)
# cap.set(4, frameHeight)
# cap.set(10,150)

# while cap.isOpened():
#     success, img = cap.read()
#     if success:
        
#         new_img = cameraCallback(img)
#         cv2.imshow("Result", new_img)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break



camera = CSICamera(width=328, height=246, capture_fps=2)
camera.running = True
camera.observe(cameraCallback, names='value')

while True:
    try:
        print('running')
        fuck = img_que.get()
        print(fuck.shape)
        cv2.imshow('test_windows',fuck)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except Exception as e:
        logging.error(traceback.format_exc())

# image = '/Users/yazhoujiang/Downloads/dog4.png'
# image = cv.imread(image)
# print('image shape')
# print(image.shape)

# batch = [image]
# data = np.asarray(batch)
# result = model(tf.convert_to_tensor(data))
# box_tensor = result['detection_boxes']
# print(box_tensor.shape)
# box_data = box_tensor[0]
# i = box_data[0]
# print("bounding box")
# print(i)
# image = cv.rectangle(image, (int(i[1]*image.shape[1]), int(i[0]*image.shape[0])),
#                      (int(i[3]*image.shape[1]), int(i[2]*image.shape[0])), (255, 0, 0), 3)
# print('detection scores:')
# print(result['detection_scores'])
# print('anchor index:')
# print(result['detection_anchor_indices'])
# cv.imshow("after", image)
# cv.waitKey(0)
# cv.destroyAllWindows()
