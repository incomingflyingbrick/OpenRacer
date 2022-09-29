from jetcam.csi_camera import CSICamera
import tensorflow_hub as hub
import tensorflow as tf
#import cv2 as cv
import numpy as np

print('model loading')
model = hub.load(
    "https://hub.tensorflow.google.cn/tensorflow/ssd_mobilenet_v2/2")
print('model success load')

def cameraCallback(change):
    # self.get_logger().info(str(change['new']))
    batch = [change['new']]
    data = np.asarray(batch)
    result = model(tf.convert_to_tensor(data))
    box_tensor = result['detection_boxes']
    class_tensor = result['detection_classes']
    print('class shape:')
    print(class_tensor.shape)
    score_tensor = result['detection_scores']
    print('score shape:')
    print(score_tensor.shape)

    print(box_tensor.shape)
    box_data = box_tensor[0]
    i = box_data[0]
    print("bounding box")
    print(i)
    print("inference done")

camera = CSICamera(width=328, height=246, capture_fps=10)
camera.running = True
camera.observe(cameraCallback, names='value')

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
