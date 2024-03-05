#!/usr/bin/env python

import rospy
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image, PointCloud2
from control_msgs.msg import PointHeadActionGoal
from geometry_msgs.msg import Pose

rospy.init_node('some_little_guy')


def detection():
    image = rospy.wait_for_message('xtion/rgb/image_raw', Image)

    request = YoloDetectionRequest()
    request.image_raw = image # sensor_msgs/Image ##subscribe to camera stuff
    request.dataset = "yolov8n.pt" # YOLOv8 model, auto-downloads
    request.confidence = 0.5 # minimum confidence to include in results
    request.nms = 0.3 # non maximal supression

    return request


def look_to_point(point):
    LOOK_TO_POINT_AS_GOAL_TOPIC = '/head_controller/point_head_action/goal'
    pub_head_topic = rospy.Publisher(LOOK_TO_POINT_AS_GOAL_TOPIC, PointHeadActionGoal, queue_size=1)

    r = rospy.Rate(10) 
    phag = PointHeadActionGoal()
    phag.header.frame_id = "/base_link"
    phag.goal.max_velocity = 1.0
    phag.goal.min_duration = rospy.Duration(0.2)
    phag.goal.target.header.frame_id = "/base_link"
    phag.goal.pointing_axis.x = 1.0
    phag.goal.pointing_frame = "/head_2_link"
    
    
    while not rospy.is_shutdown():
        phag.goal.target.point = point.position
        pub_head_topic.publish(phag)
        r.sleep()


        image = rospy.wait_for_message('xtion/rgb/image_raw', Image)
        request = YoloDetectionRequest()
        request.image_raw = image # sensor_msgs/Image ##subscribe to camera stuff
        request.dataset = "yolov8n-seg.pt" # YOLOv8 model, auto-downloads
        request.confidence = 0.2 # minimum confidence to include in results
        request.nms = 0.3 # non maximal supression



        response = detect_service(request)

        print(response)

        for detection in response.detected_objects:
            print(detection.name , detection.xywh, detection.xyseg)

            # if detection.name == "bowl":
            #     # print("AHHHHHHHHHHHHHHHHHHHHHHHHHH")
            #     print(detection.name , detection.xywh)
            #     # print(pcl)


# create service proxy 
detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
# image = rospy.wait_for_message('xtion/rgb/image_raw', Image)
# pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)


pick_pose = Pose()
pick_pose.position.x = 0.8
pick_pose.position.y = -0
pick_pose.position.z = 0.6
look_to_point(pick_pose)
