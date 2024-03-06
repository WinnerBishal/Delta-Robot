from ultralytics import YOLO
import cv2
import numpy as np
from essentials import *

import cv2

import time

tic = time.time()
#Training the model

# model = YOLO('yolov8n.pt')
# model.train(data = 'config.yaml', epochs = 100)
# Load the YOLOv8 model
model = YOLO('./runs/100_epoch/train/weights/best.pt')
results = model.predict('./tomato_images/T21.jpg', max_det = 3)
annotated_image = cv2.resize(results[0].plot(), (384, 640))
toc = time.time()
print(f"Time taken for inference: {toc - tic}")
print(results[0].boxes.xyxy[0])

# Determine object centers
object_centers = []
for box in results[0].boxes.xyxy:
    object_center = [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]
    object_centers.append(object_center)

# convert tensors to np array and convert to shape (n, 2)
object_centers = np.array(object_centers)
object_centers = object_centers.reshape(-1, 2)
for object in object_centers:
    robot_center = image2Dworld3D([object])
    # print(robot_center)
all_objects = {}
for i, label in enumerate(results[0]):
    all_objects[f'Obj{i}'] = {
        'label': label,
        'location': object_centers[i]
    }

print(all_objects['Obj0']['location'])

cv2.imshow('annotated_image.jpg', annotated_image)
cv2.waitKey(0)
cv2.destroyAllWindows()