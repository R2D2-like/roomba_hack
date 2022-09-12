#!/usr/bin/env python3

import rospy
import tf2_ros
import actionlib
import tf
from geometry_msgs.msg import Quaternion
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


import json
import os

import copy
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import PIL
import numpy as np
import rospy
import torch
import torchvision
import torchvision.transforms 


from sensor_msgs.msg import Image
from project.srv import GetGoalPoint
from project.srv import GetGoalPointResponse

class DetermineGoal:
    def __init__(self):
        rospy.init_node('determine_target_goal', anonymous=True)

        # service
        self.wave_detection_service = rospy.Service('/get_coordinate', GetGoalPoint, self.determine_goal)

    def determine_goal(self, req):

        estimete_x_min = 1.3
        estimete_x_max = 1.7
        estimete_x = 1.5

        res = GetGoalPointResponse()
        rate = rospy.Rate(10)

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        #t = rospy.Time.now()

        x = 0
        y = 0
        cnt = 0

        while cnt < 10:
            try:
                t = tfBuffer.lookup_transform('map','object_frame',rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rate.sleep()
                continue

            print('{0:.2f}, {1:.2f}, {2:.2f}'.format(
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ))
            print('{0:.2f}, {1:.2f}, {2:.2f}, {3:.2f}'.format(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ))
            x += t.transform.translation.x
            y += t.transform.translation.y
            cnt += 1

            rate.sleep()
      
        if (x/cnt<estimete_x_min) or (estimete_x_max<x/cnt):
            res.x = estimete_x
        else:
            res.x = x/cnt

        res.y = y/cnt

        return res


if __name__ == '__main__':
    dg = DetermineGoal()
    rospy.spin()


