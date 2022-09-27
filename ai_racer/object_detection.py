import tensorflow_hub as hub
import tensorflow as tf
import cv2 as cv
import numpy as np


model = hub.load("/Users/yazhoujiang/Downloads/retinanet_resnet101_v1_fpn_640x640_1")
image = '/Users/yazhoujiang/Downloads/dog4.png'
image = cv.imread(image)
print('image shape')
print(image.shape)

batch = [image]
data = np.asarray(batch)
result = model(tf.convert_to_tensor(data))
box_tensor = result['detection_boxes']
print(box_tensor.shape)
box_data = box_tensor[0]
i = box_data[0]
print("bounding box")
print(i)
image = cv.rectangle(image, (int(i[1]*image.shape[1]), int(i[0]*image.shape[0])), (int(i[3]*image.shape[1]), int(i[2]*image.shape[0])), (255,0,0), 3)
print('detection scores:')
print(result['detection_scores'])
print('anchor index:')
print(result['detection_anchor_indices'])
cv.imshow("after",image)
cv.waitKey(0)
cv.destroyAllWindows()
