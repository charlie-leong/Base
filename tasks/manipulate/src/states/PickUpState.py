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
        removeBehindTable(self.move_group)
        rospy.sleep(2)

        self.object_pose = ud.coord_input
        object_type = "top" #top, side, down

        if object_type == "down":
            self.grap_quat = Quaternion(0.5, 0.5, 0.5, 0.5)
            self.lowered_approach_grasping()
        elif object_type == "forward":
            self.forward_approach_grasping()
        elif object_type == "top":
            self.top_down_approach_grasping()

        #close gripper
        rospy.loginfo('PickObject - closing gripper!')
        close_gripper()

        raised = lift_item(self.object_pose, self.move_group)
        # place(raised, self.move_group)
        # open_gripper()
        return 'suceeded'

    def lowered_approach_grasping(self):
        '''
            for longer objects that would benefit from the gripper moving downward around it
            eg. cylinder, mug, bottle

            goes to pregrasp pose above the item
            lowers so the grippers are around the item
        '''
        # "test": [0.65, -0.04998, 0.86445],
        grasp_pose = self.object_pose
        grasp_pose.position.y -= 0.2
        grasp_pose.position.z += 0.2
        grasp_pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)

        rospy.loginfo('pre grasp pose')
        self.move_group.set_pose_target(grasp_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)

        grasp_pose.position.z -= 0.2

        rospy.loginfo('grasp pose')
        self.move_group.set_pose_target(grasp_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        rospy.sleep(2)
    
    def forward_approach_grasping(self):
        rospy.sleep(2)
    
    def top_down_approach_grasping(self):
        # grasp_pose = Pose()
        grasp_pose = self.object_pose
        # grasp_pose.position.x -= 0.2
        grasp_pose.position.y -= 0.1
        grasp_pose.position.z += 0.3
        grasp_pose.orientation = Quaternion(-0.5, 0.5, 0.5, 0.5) # pointing downwards !!!

        rospy.loginfo('pre grasp pose')
        self.move_group.set_pose_target(grasp_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(3)

        grasp_pose.position.z -= 0.1

        rospy.loginfo('grasp pose')
        self.move_group.set_pose_target(grasp_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        rospy.sleep(2)




def open_gripper():
    pub_gripper_controller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)

    # loop continues until the grippers are opened
    for i in range(10):
        trajectory = JointTrajectory()

        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

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
        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        trajectory_points = JointTrajectoryPoint()

        # define the gripper joints configuration
        trajectory_points.positions = [0.0, 0.0]

        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)

        rospy.sleep(0.1)

def removeBehindTable(move_group):
    """
        moves robot arm from the tucked position behind the table
        allows it to move more freely without colliding with table top
    """

    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = -0.2
    pose.position.z = 0.9
    pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)

    rospy.loginfo('PickObject - attempting to reach pre grasp pose')

    move_group.set_pose_target(pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(2)


def lift_item(object_pose, move_group):

    grasp_pose = object_pose
    grasp_pose.position.z += 0.3
    grasp_pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)

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
    place_pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
    move_group.set_pose_target(place_pose)
    move_group.go(wait=True)

    rospy.sleep(1)

    rospy.loginfo('place? done B)')
