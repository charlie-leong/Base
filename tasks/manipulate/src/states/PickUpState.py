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
                             input_keys=['coord_data']
                             )
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander('arm_torso')
        self.pub_gripper_controller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)


    def execute(self, ud):
        self.open_gripper()
        self.removeBehindTable()
        rospy.sleep(2)

        # self.object_pose = ud.coord_input
        self.object_pose = ud.coord_data
        
        # object_type = self.determine_object_type()
        object_type = "forward" #top, side, down
        grasp_quat = Quaternion(0.5, 0.5, 0.5, 0.5) # starting quat that the robot has

        if object_type == "down":
            grasp_quat = self.lowered_approach_grasping()
        elif object_type == "forward":
            grasp_quat = self.forward_approach_grasping()
        elif object_type == "top":
            grasp_quat = self.top_down_approach_grasping()

        #close gripper
        rospy.loginfo('PickObject - closing gripper!')
        self.close_gripper()

        self.lift_item(grasp_quat)
        # place(raised, self.move_group)
        # open_gripper()
        return 'suceeded'

    def send_pose(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def lowered_approach_grasping(self):
        '''
            for longer objects that would benefit from the gripper moving downward around it
            eg. cylinder, mug, bottle

            goes to pregrasp pose above the item
            lowers so the grippers are around the item
        '''
        grasp_quat = Quaternion(0.5, 0.5, 0.5, 0.5)

        # "test": [0.65, -0.04998, 0.86445],
        grasp_pose = self.object_pose
        grasp_pose.position.y -= 0.2
        grasp_pose.position.z += 0.2
        grasp_pose.orientation = grasp_quat

        rospy.loginfo('pre grasp pose')
        self.send_pose(grasp_pose)

        grasp_pose.position.z -= 0.2

        rospy.loginfo('grasp pose')
        self.send_pose(grasp_pose)

        rospy.sleep(2)
        
        return grasp_quat
    
    def forward_approach_grasping(self):
        '''
        for flatter objects that can be picked up from the front onwards
        depending on the object's shape this may be contingent on there being a slight overhang on the table
        otherwise may simply push the object forward
        '''
        rot_quat = Quaternion(0.7, 0.7, 0, 0)  # 90* around X axis (W, X, Y, Z)
        grasp_quat = self.quaternion_multiply(rot_quat, Quaternion(0.5, 0.5, 0.5, 0.5))

        grasp_pose = self.object_pose
        grasp_pose.position.x -= 0.4
        grasp_pose.position.z += 0.05
        grasp_pose.orientation = grasp_quat
        print(grasp_pose)

        self.send_pose(grasp_pose)

        grasp_pose.position.x += 0.1

        self.send_pose(grasp_pose)

        rospy.sleep(2)
        return grasp_quat
    
    def top_down_approach_grasping(self):
        grasp_pose = self.object_pose
        # grasp_pose.position.x -= 0.2
        grasp_pose.position.y -= 0.07
        grasp_pose.position.z += 0.3
        grasp_pose.orientation = Quaternion(-0.5, 0.5, 0.5, 0.5) # pointing downwards !!!

        rospy.loginfo('pre grasp pose')
        self.send_pose(grasp_pose)

        grasp_pose.position.z -= 0.05

        rospy.loginfo('grasp pose')
        self.send_pose(grasp_pose)

        rospy.sleep(2)
        return Quaternion(0.5, -0.5, 0.5, 0.5) # upwards

    def determine_object_type(self):
        # based on width length height its in my notion
        print("TBD")
    
    def lift_item(self, lift_quat):
        lift_pose = self.object_pose
        lift_pose.position.y = -0.1
        lift_pose.position.z += 0.2
        lift_pose.orientation = lift_quat

        # move to grasp_pose
        rospy.loginfo('up !!')
        self.send_pose(lift_pose)

        rospy.sleep(1)
        return lift_pose
        
    def quaternion_multiply(self, q0, q1):
        x0, y0, z0, w0 = q0.x, q0.y, q0.z, q0.w
        x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
        
        x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        y = w0 * y1 + y0 * w1 + z0 * x1 - x0 * z1
        z = w0 * z1 + z0 * w1 + x0 * y1 - y0 * x1
        w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        
        return Quaternion(x, y, z, w)

    def open_gripper(self):
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

            self.pub_gripper_controller.publish(trajectory)
        
            # interval to start next movement
            rospy.sleep(0.1)


    def close_gripper(self):
        # loop continues until the grippers is closed
        for i in range(10):
            trajectory = JointTrajectory()
            trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
            trajectory_points = JointTrajectoryPoint()

            # define the gripper joints configuration
            trajectory_points.positions = [0.0, 0.0]

            trajectory_points.time_from_start = rospy.Duration(1.0)

            trajectory.points.append(trajectory_points)

            self.pub_gripper_controller.publish(trajectory)

            rospy.sleep(0.1)

    def removeBehindTable(self):
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

        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def place(self, object_pose):
        rospy.loginfo('placing')

        place_pose = object_pose
        place_pose.position.z -= 0.2
        place_pose.position.y += 0.2
        place_pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
        self.move_group.set_pose_target(place_pose)
        self.move_group.go(wait=True)

        rospy.sleep(1)

        rospy.loginfo('place? done B)')
