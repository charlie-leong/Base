#!/usr/bin/env python3

import rospy
from manipulate.srv import GetItemCoords
from geometry_msgs.msg import Pose

def get_next_item(item):
    print(item)
    i = item.item
    return create_pose_from_coords(item_coords_dict[i])

def create_pose_from_coords(coords):
    object_pose = Pose()

    object_pose.position.x = coords[0]
    object_pose.position.y = coords[1]
    object_pose.position.z = coords[2]
    object_pose.orientation.w = 0
    print(object_pose)
    return object_pose

item_coords_dict = {  
    "test": [0.65, -0.04998, 0.86445],
    "cylinder": [0.537632, -0.210692, 0.626178],
    "cup": [0.6, 0, 0.815]
}

rospy.init_node("itemservice")  # create node roomservice
print("created service !")
get_point_srv = rospy.Service('point_from_item', GetItemCoords, get_next_item)
rospy.spin()
