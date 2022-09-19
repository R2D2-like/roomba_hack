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


if __name__ == '__main__':
    rospy.init_node('tf_test', anonymous=True)
    cnt =0
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)



    while not rospy.is_shutdown():
        try:
            t = tfBuffer.lookup_transform('map','base_footprint',rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rospy.sleep(0.1)
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
        cnt += 1

        rospy.sleep(0.1)