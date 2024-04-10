#!/usr/bin/env python3

import rospy
# from geometry_msgs.msg import Pose, Quaternion
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import smach

from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image, PointCloud2
from control_msgs.msg import PointHeadActionGoal


class LookForItem(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['suceeded', 'preempted'],
                             input_keys=['coord_input', 'first_item'],
                             output_keys=['seg', 'pcl']
                             )
        LOOK_TO_POINT_AS_GOAL_TOPIC = '/head_controller/point_head_action/goal'
        self.pub_head_topic = rospy.Publisher(LOOK_TO_POINT_AS_GOAL_TOPIC, PointHeadActionGoal, queue_size=1)

    def execute(self, ud):
        self.detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.look_to_point(ud, ud.coord_input)
        return 'suceeded'
    
    # def look_to_point(self, ud):
    #     r = rospy.Rate(10) 
    #     phag = PointHeadActionGoal()
    #     phag.header.frame_id = "/base_link"
    #     phag.goal.max_velocity = 1.0
    #     phag.goal.min_duration = rospy.Duration(0.2)
    #     phag.goal.target.header.frame_id = "/base_link"
    #     phag.goal.pointing_axis.x = 1.0
    #     phag.goal.pointing_frame = "/head_2_link"
    #     found = False
    #     while found == False:
    #         phag.goal.target.point = ud.coord_input.position
    #         self.pub_head_topic.publish(phag)
    #         r.sleep()
    #         found = self.detect(ud)

    def look_to_point(self, ud, point):
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
            pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
            request = YoloDetectionRequest()
            request.image_raw = image # sensor_msgs/Image ##subscribe to camera stuff
            request.dataset = "yolov8n-seg.pt" # YOLOv8 model, auto-downloads
            request.confidence = 0.2 # minimum confidence to include in results
            request.nms = 0.3 # non maximal supression

            response = self.detect_service(request)
            # print(response)
            # return "suceeded"
    
            for detection in response.detected_objects:
                if detection.name == "scissors":#ud.first_item:
                    print(detection.name , detection.xywh)
                    ud.seg = detection.xyseg
                    ud.pcl = pcl
                    return "suceeded"




            
    # def detect(self, ud):
    #     detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
    #     image = rospy.wait_for_message('xtion/rgb/image_raw', Image)
    #     pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

    #     # create request
    #     request = YoloDetectionRequest()
    #     request.image_raw = image # sensor_msgs/Image ##subscribe to camera stuff
    #     request.dataset = "yolov8n.pt" # YOLOv8 model, auto-downloads
    #     request.confidence = 0.5 # minimum confidence to include in results
    #     request.nms = 0.3

    #     response = detect_service(request)
    #     for detection in response.detected_objects:

    #         print(detection.name, detection.xywh)
    #         print(detection.name, detection.xyseg)
    #         if detection.name == ud.first_object:

                # ud.coord_data = detection.xyseg
                # ud.pcl = pcl
    #             return True
    #     return False



