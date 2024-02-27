#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import sys
import smach

class PickUpItem(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['suceeded', 'preempted'],
                             input_keys=['coord_input']
                             )
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander('arm_torso')


    def execute(self, ud):
        open_gripper()

        object_pose = ud.coord_input

        goToObject(object_pose, self.move_group)
        rospy.sleep(2)
        pre_pick(object_pose, self.move_group)
        rospy.sleep(2)
        grasp = pick(object_pose, self.move_group)
        raised = lift_item(grasp, self.move_group)
        place(raised, self.move_group)
        open_gripper()
        return 'suceeded'

def open_gripper():
    pub_gripper_controller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)

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

def goToObject(object_pose, move_group):
    """
        Puts the manipulator in the right position to take the object
    Args:
        object_pose (msg): get object pose

    Returns:
        response: operation success status
    """

    # define final orientation of the EE
    object_pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)

    # set vertical pose of the EE taking care of its shape
    object_pose.position.z += 0.2

    # save grasp pose
    # grasp_pose = copy.deepcopy(object_pose.pose)
    grasp_pose = object_pose

    # define distance between EE & object
    grasp_pose.position.y -= 0.35
    grasp_pose.position.x -= 0.1

    # for some reason the code hinges on the existence of this line ??
    object_pose.position.y = -0.2

    # grasp_pose.position.z += 0.2

    rospy.loginfo('PickObject - attempting to reach pre grasp pose')

    move_group.set_pose_target(object_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    # object_pose.position.y = -0.2

def pre_pick(object_pose, move_group):
    open_gripper()

    grasp_pose = object_pose
    grasp_pose.position.x += 0.1
    grasp_pose.position.y -=0.05

    # move to grasp_pose
    rospy.loginfo('PickObject - move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.sleep(1)

def pick(object_pose, move_group):
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

    return grasp_pose

    # rospy.loginfo('lifting !')

    # define post_grasp_pose

    # post_grasp_pose = object_pose
    # post_grasp_pose.position.z += 0.3
    # move_group.set_pose_target(post_grasp_pose)
    # move_group.go(wait=True)

    # rospy.sleep(1)

    rospy.loginfo('PickObject - Done.')


def lift_item(object_pose, move_group):

    grasp_pose = object_pose
    grasp_pose.position.z += 0.3

    # move to grasp_pose
    rospy.loginfo('up !!')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.sleep(1)
    return grasp_pose

def place(object_pose, move_group):
    rospy.loginfo('placing')

    place_pose = object_pose
    place_pose.position.z -= 0.2
    place_pose.position.y += 0.2
    move_group.set_pose_target(place_pose)
    move_group.go(wait=True)

    rospy.sleep(1)

    rospy.loginfo('place? done B)')
