#!/usr/bin/env python3

import rospy
# from geometry_msgs.msg import Pose, Quaternion
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import smach

from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
# from sensor_msgs.msg import Image, PointCloud2, point_cloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Pose, PointStamped, Point, PoseStamped
from control_msgs.msg import PointHeadActionGoal

import tf2_ros
from tf.transformations import *
from tf2_geometry_msgs import do_transform_pose
from tf_module.srv import TfTransform, TfTransformRequest



class GetObjectInformaion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['suceeded', 'preempted'],
                             input_keys=['coord_data', 'pcl'],
                             output_keys=['coord_data', "object_type"]
                             )
        LOOK_TO_POINT_AS_GOAL_TOPIC = '/head_controller/point_head_action/goal'
        self.pub_head_topic = rospy.Publisher(LOOK_TO_POINT_AS_GOAL_TOPIC, PointHeadActionGoal, queue_size=1)

    def execute(self, ud):
        pcl = ud.pcl
        relative_coords = self.estimate_coords(ud, ud.coord_data, pcl)
        actual = self.translate_coord_to_map(relative_coords, pcl.header)
        return 'suceeded'
    
    def estimate_coords(self, ud, seg, pc):
        depth_data = point_cloud2.read_points(pc, field_names=("x", "y", "z"), skip_nans=True)
        depth_array = np.array(list(depth_data))

        xs = []
        ys = []
        zs = []
        for coord in seg:
            xs.append(depth_array[coord][0])
            ys.append(depth_array[coord][1])
            zs.append(depth_array[coord][2])

        # Calculate the average depth
        average_depth_x = np.mean(xs)
        average_depth_y = np.mean(ys)
        average_depth_z = np.mean(zs)
        self.estimate_orientation(ud, xs, ys, zs)

        return (average_depth_x, average_depth_y, average_depth_z)


    def translate_coord_to_map(self, ud, coords, header):
        tf_service = rospy.ServiceProxy('tf_transform', TfTransform)
        x, y, z = coords

        point = PointStamped()
        point.point = Point(x, y, z)
        point.header = header
        
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = point

        response = tf_service(tf_req)
        # print(response)

        return response.target_point.point.x, response.target_point.point.y
    
    def estimate_orientation(self, ud, xs, ys, zs):
        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        z_range = max(zs) - min(zs)
        # print(x_range, y_range, z_range)
        self.determine_object_type(x_range, y_range, z_range)

        return (x_range, y_range, z_range)
    
    def determine_object_type(self, ud, x, y, z):
        gripper_width = 0.5
        if(y <= gripper_width):
            ud.object_type = 'down'
        elif(x ):
            ud.object_type = 'forward'
        elif(z):
            ud.object_type = 'top'
        else:
            print("no valid grasping approach available")
            return 'aborted'
        return 'suceeded'