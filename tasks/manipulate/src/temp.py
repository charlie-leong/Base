#!/usr/bin/env python

import rospy
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image, PointCloud2

# create service proxy
detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

image = rospy.wait_for_message('xtion/rgb/image_raw', Image)

# create request
request = YoloDetectionRequest()
request.image_raw = image # sensor_msgs/Image ##subscribe to camera stuff
request.dataset = "yolov8n.pt" # YOLOv8 model, auto-downloads
request.confidence = 0.5 # minimum confidence to include in results
request.nms = 0.3 # non maximal supression

# send request
response = detect_service(request)
# .. use request.detections 
# DON"T ??????

for detection in response.detected_objects:
    if detection.name == "mug":
        print("AHHHHHHHHHHHHHHHHHHHHHHHHHH")