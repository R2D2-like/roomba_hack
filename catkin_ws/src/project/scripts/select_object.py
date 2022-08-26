#!/usr/bin/env python3

import rospy
from project.msg import StringArray
from project.srv import Object_List
from project.srv import Object_ListResponse
from std_msgs.msg import String
#import sys

# def select_object(req):
#     path = "/root/roomba_hack/catkin_ws/src/three-dimensions_tutorial/yolov3/"

#     # load category
#     category_list =[]
#     with open(path+"data/coco.names") as f:
#         category = f.read().splitlines()

#     for val in req.detobject:
#         category_list.append(category[val])

#     target = input("Which object do you choose amoung" + str(category_list) + "?:") # we need '' when we asked to write
#     return Object_ListResponse(tarobject=target)

def select_object(req):

    target = input("Which object do you choose amoung" + str(req.detobject) + "?:") # we need not '' when we asked to write
    return Object_ListResponse(tarobject=target)

if __name__ == "__main__":
    rospy.init_node("select_object")
    select_ob_server = rospy.Service("select_object", Object_List, select_object)
    rospy.spin()
