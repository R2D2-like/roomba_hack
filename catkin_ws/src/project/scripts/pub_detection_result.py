#!/usr/bin/env python3

from os import TMP_MAX
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pytorchyolo import detect, models
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import copy
import numpy as np
 
class detection_result:
    def __init__(self):
        rospy.init_node('pub_detection_result', anonymous=True)
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)
        rgb_sub =   rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.bridge = CvBridge()
        self.rgb_image = None


    def callback(self, image):
        print('callback')
        cv_array = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array
        

    def process(self):
        path = "/root/roomba_hack/catkin_ws/src/three-dimensions_tutorial/yolov3/"

        # load category
        with open(path+"data/coco.names") as f:
            category = f.read().splitlines()

        # prepare model
        model = models.load_model(path+"config/yolov3.cfg", path+"weights/yolov3.weights")

        while not rospy.is_shutdown():
            print('l35')
            if self.rgb_image is None:
                
                continue

            # inference
            tmp_image = copy.copy(self.rgb_image)

            boxes = detect.detect_image(model, tmp_image)
            # [[x1, y1, x2, y2, confidence, class]]

            # plot bouding box
            for box in boxes:
                print('debug')
                x1, y1, x2, y2 = map(int, box[:4])
                cls_pred = int(box[5])

                tmp_image = cv2.rectangle(tmp_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                tmp_image = cv2.putText(tmp_image, category[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cx, cy = (x1+x2)//2, (y1+y2)//2
                print(category[cls_pred])

            # publish image

            tmp_image = cv2.cvtColor(tmp_image, cv2.COLOR_RGB2BGR)
            detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            self.detection_result_pub.publish(detection_result)
      

if __name__ == '__main__':
    dr = detection_result()
    try:
        dr.process()
    except rospy.ROSInitException:
        pass
