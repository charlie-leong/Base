#!/usr/bin/python

import rospy
import smach
# import actionlib
from manipulate.srv import GetItemCoords
from smach_ros import ServiceState
from states.PickUpState import PickUpItem
# from states.GoToState import GoToPoint


def main():
    rospy.init_node('main_node')
    get_point = rospy.ServiceProxy('point_from_item', GetItemCoords)
    get_point.wait_for_service()
    rospy.logwarn('THIS IS BEING REACHED')
    # robot = Default()

    sm = smach.StateMachine(outcomes=['success', 'aborted', 'finished'])

    with sm:
        smach.StateMachine.add('GET_ITEM_COORDS', ServiceState('point_from_item', GetItemCoords, 
                                    request_slots=['item'], response_slots=['coordinates']),
                                    transitions={'succeeded': 'PICK_UP_ITEM', 'preempted': 'aborted'},
                                    remapping={'item': 'first_item', 'coordinates': 'coord_data'}
                                    )  

        # smach.StateMachine.add('GO_TO_ITEM', GoToPoint(robot), 
        #                        transitions={'suceeded': 'finished', 'preempted': 'aborted'},
        #                        remapping={'coord_input': 'coord_data'}
        #                        )

        smach.StateMachine.add('PICK_UP_ITEM', PickUpItem(), 
                               transitions={'suceeded': 'finished', 'preempted': 'aborted'},
                               remapping={'coord_input': 'coord_data'}
                               )

    sm.userdata.first_item = 'cup'
    sm.execute()

if __name__ == '__main__':
    main()
