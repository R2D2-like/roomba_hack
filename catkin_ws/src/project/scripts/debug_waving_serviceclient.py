#!/usr/bin/env python3

import rospy
from project.srv import WavingLeftRight

if __name__ == '__main__':
    rospy.init_node('waving_client')
    person = rospy.ServiceProxy('/wave_detection',WavingLeftRight)
    res = person()

    print(res.left_or_right)