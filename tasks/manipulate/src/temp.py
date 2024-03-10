#!/usr/bin/env python

import rospy
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from control_msgs.msg import PointHeadActionGoal
from geometry_msgs.msg import Pose, PointStamped, Point, PoseStamped
import numpy as np
from tf_module.srv import TfTransform, TfTransformRequest
from std_msgs.msg import String


import tf2_ros
from tf.transformations import *
from tf2_geometry_msgs import do_transform_pose


rospy.init_node('some_little_guy')


def detection():
    image = rospy.wait_for_message('xtion/rgb/image_raw', Image) 

    request = YoloDetectionRequest()
    request.image_raw = image # sensor_msgs/Image ##subscribe to camera stuff
    request.dataset = "yolov8n.pt" # YOLOv8 model, auto-downloads
    request.confidence = 0.5 # minimum confidence to include in results
    request.nms = 0.3 # non maximal supression

    return request

def estimate_coords(seg, pc):
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
    estimate_orientation(xs, ys, zs)

    return (average_depth_x, average_depth_y, average_depth_z)

def estimate_orientation(xs, ys, zs):
    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    z_range = max(zs) - min(zs)
    # print(x_range, y_range, z_range)
    return (x_range, y_range, z_range)
    #hammer
    # 1.9965749979019165 0.0038220882415771484 0.008403778076171875
    #bowl
    # 2.0788851976394653 0.00382232666015625 0.008334159851074219

def translate_coord_to_map(coords, header):
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

def relative_to_absolute(relative_pose, header):
    # print(header.frame_id)

    tfBuffer = tf2_ros.Buffer()

    ps = PoseStamped()
    
    ps.pose.position = relative_pose
    ps.header.stamp = rospy.Time().now() # tfBuffer.get_latest_common_time("base_footprint", 'odom')
    ps.header.frame_id = header.frame_id
    transform_ok = False
    while not transform_ok and not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform("base_footprint", 
                                    "odom",
                                    rospy.Time(0))
            abs_pose = do_transform_pose(ps, transform)
            print(abs_pose)
            transform_ok = True
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr("uh oh")

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
        pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        request = YoloDetectionRequest()
        request.image_raw = image # sensor_msgs/Image ##subscribe to camera stuff
        request.dataset = "yolov8n-seg.pt" # YOLOv8 model, auto-downloads
        request.confidence = 0.2 # minimum confidence to include in results
        request.nms = 0.3 # non maximal supression


        response = detect_service(request)
        

        print(response)

        for detection in response.detected_objects:
            # print(detection.name , detection.xywh, detection.xyseg)
            AH = estimate_coords(detection.xyseg, pcl)
            print("rel = ", AH)
            
            # relative_to_absolute(AH, pcl.header)

            actual = translate_coord_to_map(AH, pcl.header)
            print("actual coordinates = " , actual)

            ITR_w7(AH)
            # return actual  

            # if detection.name == "bowl":
            #     # print("AHHHHHHHHHHHHHHHHHHHHHHHHHH")
            #     print(detection.name , detection.xywh)
            #     # print(pcl)




# create service proxy 
detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
# pose = rospy.Subscriber('')
# image = rospy.wait_for_message('xtion/rgb/image_raw', Image)
# pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)


pick_pose = Pose()
pick_pose.position.x = 0.4
pick_pose.position.y = -0
pick_pose.position.z = 0.6
look_to_point(pick_pose)
