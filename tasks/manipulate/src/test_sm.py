#!/usr/bin/python
import rospy
from state_machine import main as run_state_machine

if __name__ == '__main__':
    rospy.init_node('main_node')
    run_state_machine()


