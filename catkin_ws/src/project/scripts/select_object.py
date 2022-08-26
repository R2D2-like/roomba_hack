#!/usr/bin/env python3

import rospy
from project.msg import StringArray
from project.srv import Object_List
from project.srv import Object_ListResponse
from std_msgs.msg import String
#import sys

def select_object(req):
    target = input("Which object do you choose amoung" + str(req.detobject) + "?:") # we need '' when we asked to write
    return Object_ListResponse(tarobject=target)

if __name__ == "main":
    rospy.init_node("select_object")
    select_ob_server = rospy.Service("select_object", Object_List, select_object)
    rospy.spin()
