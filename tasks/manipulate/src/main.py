#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Quaternion

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import sys

from tf.transformations import quaternion_multiply, quaternion_about_axis
from math import pi

import numpy as np
from scipy.spatial.transform import Rotation 
import math

def open_gripper():
    pub_gripper_controller = rospy.Publisher(
        '/gripper_controller/command', JointTrajectory, queue_size=1)

   # loop continues until the grippers is opened
    for i in range(10):
        trajectory = JointTrajectory()

        # call joint group for take object
        trajectory.joint_names = [
            'gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajectory_points = JointTrajectoryPoint()

        # define the gripper joints configuration
        trajectory_points.positions = [0.044, 0.044]

        # define time duration
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)
        
        # interval to start next movement
        rospy.sleep(0.1)


def close_gripper():
    # publish gripper status on joint trajectory when TIAGo close gripper
    pub_gripper_controller = rospy.Publisher(
        '/gripper_controller/command', JointTrajectory, queue_size=1)

    # loop continues until the grippers is closed
    for i in range(10):
        trajectory = JointTrajectory()

        # call joint group for take object
        trajectory.joint_names = [
            'gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajectory_points = JointTrajectoryPoint()

        # define the gripper joints configuration
        trajectory_points.positions = [0.0, 0.0]

        # define time duration
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)

        # interval to start next movement
        rospy.sleep(0.1)

def goToObject(object_pose):
    """
        Puts the manipulator in the right position to take the object
    Args:
        object_pose (msg): get object pose

    Returns:
        response: operation success status
    """
    # get global variable
    global move_group
    global grasp_poserad

    # define final orientation of the EE
    object_pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)

    # set vertical pose of the EE taking care of its shape
    # object_pose.position.z += 0.2

    # save grasp pose
    grasp_pose = object_pose

    # define distance between EE & object
    # grasp_pose.position.y -= 0.15
    grasp_pose.position.x -= 0.1

    # for some reason the code hinges on the existence of this line ??
    object_pose.position.y -= 0.15

    rospy.loginfo('PickObject - attempting to reach pre grasp pose')

    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    # print("go to object", object_pose)
    # print("go to object", grasp_pose)
    # object_pose.position.y = -0.2

def pre_pick(object_pose):
    open_gripper()

    grasp_pose = object_pose
    grasp_pose.position.x += 0.1
    grasp_pose.position.y -= 0.15

    # move to grasp_pose
    rospy.loginfo('PickObject - move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    # print("pre pick", object_pose)

    rospy.sleep(1)

def pick(object_pose):
    grasp_pose = object_pose
    # grasp_pose.position.x += 0.05
    grasp_pose.position.z -= 0.2

    # move to grasp_pose
    rospy.loginfo('PickObject - move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    #close gripper
    rospy.loginfo('PickObject - closing gripper...')
    close_gripper()
    rospy.sleep(2)
    # print("pick", object_pose)


def lift(object_pose):
    rospy.loginfo('lifting !')

    post_grasp_pose = object_pose
    post_grasp_pose.position.z += 0.3
    move_group.set_pose_target(post_grasp_pose)
    move_group.go(wait=True)

    rospy.sleep(1)

    rospy.loginfo('PickObject - Done.')
    # print("lift", object_pose)


def place(object_pose):
    rospy.loginfo('placing')

    place_pose = object_pose
    place_pose.position.z -= 0.3
    move_group.set_pose_target(place_pose)
    move_group.go(wait=True)

    rospy.sleep(1)

    rospy.loginfo('place? done B)')
    # print("place", object_pose)

rospy.init_node('main_node')


print("AH")

open_gripper()





def rotate_quaternion_y(q, angle):
    half_angle = angle / 2
    sin_half_angle = math.sin(half_angle)
    cos_half_angle = math.cos(half_angle)

    x = 0
    y = sin_half_angle
    z = 0
    w = cos_half_angle

    rotation_quaternion = Quaternion(x, y, z, w)
    print(rotation_quaternion)
    return quaternion_multiply(q, rotation_quaternion)

def quaternion_multiply(q0, q1):
    x0, y0, z0, w0 = q0.x, q0.y, q0.z, q0.w
    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    
    x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    y = w0 * y1 + y0 * w1 + z0 * x1 - x0 * z1
    z = w0 * z1 + z0 * w1 + x0 * y1 - y0 * x1
    w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    
    return Quaternion(x, y, z, w)

pose_1 = Pose()
pose_1.position.x = 0.6
pose_1.position.y = -0.05
pose_1.position.z = 0.65+0.3
# pose_1.orientation.w = 0
# pose_1.orientation = Quaternion(-0.5, 0.5, 0.5, 0.5) # pointing downwards !!!
# pose_1.orientation = Quaternion(0.5, -0.5, 0.5, 0.5) # upwards
# pose_1.orientation = Quaternion(-0.5, 0.5, -0.5, -0.5) # also upwards
pose_1.orientation = Quaternion(-0.5, 0.5, 0.5, 0.5)
# pose_1.orientation = desired_quaternion.as_quat() #Quaternion(desired_quaternion[0], desired_quaternion[1], desired_quaternion[2], desired_quaternion[3])


object_pose = Pose()
object_pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)


object_pose.position.z = 0.9
object_pose.position.x = 0.6-0.1
# for some reason the code hinges on the existence of this line ??
object_pose.position.y = -0.2

moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander('arm_torso')

 # behind table pose
move_group.set_pose_target(object_pose)
move_group.go(wait=True)
rospy.sleep(3)

#hammer
# move_group.set_pose_target(pose_1)
# move_group.go(wait=True)
# rospy.sleep(3)

# pose_1.position.z -=0.12
# move_group.set_pose_target(pose_1)
# move_group.go(wait=True)
# rospy.sleep(3)


#rotate
rot_quat = Quaternion(0.7, 0.7, 0, 0)  # 90* around X axis (W, X, Y, Z)
grasp_quat = quaternion_multiply(rot_quat, Quaternion(0.5, 0.5, 0.5, 0.5)) #HOLY FUCKING SHIT
# print(grasp_quat)

# print("1")
# bowl_next = object_pose
# bowl_next.position.x -= 0.06
# bowl_next.orientation = grasp_quat
# move_group.set_pose_target(bowl_next)
# move_group.go(wait=True)
# rospy.sleep(3)


# hammer
# close_gripper()
# pose_pick = Pose()
# pose_pick.position.x = 0.639713
# pose_pick.position.y = -0.1
# pose_pick.position.z = 0.65+0.2
# pose_pick.orientation = Quaternion(0.5, -0.5, 0.5, 0.5) # upwards
# move_group.set_pose_target(pose_pick)
# move_group.go(wait=True)
# rospy.sleep(3)


# angle_to_rotate = math.radians(1)  # Rotate by 1 degree (very slight rotation)
# rotated_quaternion = rotate_quaternion_y(grasp_quat, angle_to_rotate)
# bowl_next.orientation = rotated_quaternion
# move_group.set_pose_target(bowl_next)
# move_group.go(wait=True)
# rospy.sleep(3)


#bowl
print("2")
pose = Pose()
pose.position.x = 0.49
pose.position.y = -0
pose.position.z = 0.66
pose.orientation = grasp_quat

move_group.set_pose_target(pose)
move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()
rospy.sleep(2)

print("3")
pick_pose = Pose()
pick_pose.position.x = 0.60
pick_pose.position.y = -0
pick_pose.position.z = 0.66
pick_pose.orientation = grasp_quat

move_group.set_pose_target(pick_pose)
move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()
rospy.sleep(3)

close_gripper()
rospy.sleep(2)
# pick_pose.position.x += 0.5
pick_pose.position.z += 0.3
move_group.set_pose_target(pick_pose)
move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

# move_group.set_pose_target(pose_3)
# move_group.go(wait=True)

# goToObject(object_pose)
# rospy.sleep(2)
# pre_pick(object_pose)
# rospy.sleep(2)
# pick(object_pose)
# lift(object_pose)
# place(object_pose)
# open_gripper()
