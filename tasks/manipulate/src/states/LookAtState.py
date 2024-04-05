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
                             input_keys=['coord_input'],
                             output_keys=['coord_data']
                             )
        LOOK_TO_POINT_AS_GOAL_TOPIC = '/head_controller/point_head_action/goal'
        self.pub_head_topic = rospy.Publisher(LOOK_TO_POINT_AS_GOAL_TOPIC, PointHeadActionGoal, queue_size=1)

    def execute(self, ud):
        self.look_to_point(ud)
        return 'suceeded'
    
    def look_to_point(self, ud):
        r = rospy.Rate(10) 
        phag = PointHeadActionGoal()
        phag.header.frame_id = "/base_link"
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)
        phag.goal.target.header.frame_id = "/base_link"
        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "/head_2_link"
        found = False
        while found == False:
            phag.goal.target.point = ud.coord_input.position
            self.pub_head_topic.publish(phag)
            r.sleep()
            found = self.detect(ud)
            
            
    def detect(self, ud):
        detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        image = rospy.wait_for_message('xtion/rgb/image_raw', Image)
        pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

        # create request
        request = YoloDetectionRequest()
        request.image_raw = image # sensor_msgs/Image ##subscribe to camera stuff
        request.dataset = "yolov8n.pt" # YOLOv8 model, auto-downloads
        request.confidence = 0.5 # minimum confidence to include in results
        request.nms = 0.3

        response = detect_service(request)
        for detection in response.detected_objects:

            print(detection.name, detection.xywh)
            print(detection.name, detection.xyseg)
            if detection.name == ud.first_object:

                ud.coord_data = detection.xyseg
                ud.pcl = pcl
                return True
        return False

# def estimate_coords(seg, pc):
#     depth_data = point_cloud2.read_points(pc, field_names=("x", "y", "z"), skip_nans=True)
#     depth_array = np.array(list(depth_data))

#     xs = []
#     ys = []
#     zs = []
#     for coord in seg:
#         xs.append(depth_array[coord][0])
#         ys.append(depth_array[coord][1])
#         zs.append(depth_array[coord][2])

#     # Calculate the average depth
#     average_depth_x = np.mean(xs)
#     average_depth_y = np.mean(ys)
#     average_depth_z = np.mean(zs)
#     # estimate_orientation(xs, ys, zs)

#     return (average_depth_x, average_depth_y, average_depth_z)

