#!/usr/bin/env python3

from os import TMP_MAX
import rospy
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from pytorchyolo import detect, models
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import copy
import numpy as np
from project.msg import StringArray
from project.srv import Object_List
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class DetectionDistance:
    def __init__(self):
        rospy.init_node('detection_distance', anonymous=True)
        #action
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()  # action serverの準備ができるまで待つ

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)
        self.depth_mask_pub = rospy.Publisher('/depth_mask', Image, queue_size=10)
        self.cam_info_pub = rospy.Publisher('/camerainfo/depth_mask', CameraInfo, queue_size=10)


        # Subscriber
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        cam_info_sub = message_filters.Subscriber('/camera/color/camera_info',CameraInfo)
        message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, cam_info_sub], 10, 1.0).registerCallback(self.callback_rgbd)

        self.bridge = CvBridge()
        self.rgb_image, self.depth_image = None, None
        self.flag = True

    def callback_rgbd(self, data1, data2, data3):
        cv_array = self.bridge.imgmsg_to_cv2(data1, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

        cv_array = self.bridge.imgmsg_to_cv2(data2, 'passthrough')
        self.depth_image = cv_array

        self.cam_info = data3

    def process(self):
        path = "/root/roomba_hack/catkin_ws/src/three-dimensions_tutorial/yolov3/"

        # load category
        with open(path+"data/coco.names") as f:
            category = f.read().splitlines()

        # prepare model
        model = models.load_model(path+"config/yolov3.cfg", path+"weights/yolov3.weights")

        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue

            # inference
            tmp_image = copy.copy(self.rgb_image)
            tmp_depth = copy.copy(self.depth_image)
            tmp_caminfo = copy.copy(self.cam_info)
            print(tmp_caminfo.header)
            boxes = detect.detect_image(model, tmp_image)
            # [[x1, y1, x2, y2, confidence, class]]
            # cv_array2 = self.bridge.imgmsg_to_cv2(tmp_depth, '32FC1')
            mask = np.zeros_like(tmp_depth)
            coordinate_list = []
            class_list = []
            class_list_idx = []

            # plot bouding box
            for box in boxes:
                x1, y1, x2, y2 = map(int, box[:4])
                cls_pred = int(box[5])
                #mask[y1:y2,x1:x2] = 1

                #print(mask)

                coordinate_list.append([x1, y1, x2, y2])

                class_list.append(category[cls_pred])
                class_list_idx.append(cls_pred)

                tmp_image = cv2.rectangle(tmp_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                tmp_image = cv2.putText(tmp_image, category[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cx, cy = (x1+x2)//2, (y1+y2)//2
                print(category[cls_pred], self.depth_image[cy][cx]/1000, "m")
            
            #service
            if self.flag:
                self.action_client.cancel_goal() 
                set_object = rospy.ServiceProxy("select_object", Object_List)
                print(class_list)
                print(type(class_list))
                #rospy.sleep(30)
                res = set_object(class_list) #you can get one name of the target object 
                #res = set_object(class_list_idx)
                self.flag = False
                target_coordinate_idx = class_list.index(res.tarobject)
                target_object_name = res.tarobject
                mask[coordinate_list[target_coordinate_idx][1], coordinate_list[target_coordinate_idx][3], coordinate_list[target_coordinate_idx][0], coordinate_list[target_coordinate_idx][2]] = 1
                print(mask)
                mask_result = np.where(mask,tmp_depth,0)
                mask_result = self.bridge.cv2_to_imgmsg(mask_result, "passthrough")
                mask_result.header = tmp_caminfo.header
                self.depth_mask_pub.publish(mask_result)
            elif target_object_name in class_list: 
                target_coordinate_idx = class_list.index(target_object_name) 
                mask[coordinate_list[target_coordinate_idx][1], coordinate_list[target_coordinate_idx][3], coordinate_list[target_coordinate_idx][0], coordinate_list[target_coordinate_idx][2]] = 1
                print(mask)
                mask_result = np.where(mask,tmp_depth,0)
                mask_result = self.bridge.cv2_to_imgmsg(mask_result, "passthrough")
                mask_result.header = tmp_caminfo.header
                self.depth_mask_pub.publish(mask_result)
            else:
                self.flag = True #when the roomba loses sight of the target
                # rate = rospy.Rate(0.1)
                # rate.sleep()
                rospy.sleep(5.0)

            # publish image

            tmp_image = cv2.cvtColor(tmp_image, cv2.COLOR_RGB2BGR)
            detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            # mask_result = np.where(mask,tmp_depth,0)
            # mask_result = self.bridge.cv2_to_imgmsg(mask_result, "passthrough")
            # mask_result.header = tmp_caminfo.header
            self.detection_result_pub.publish(detection_result)
            # self.depth_mask_pub.publish(mask_result)
            self.cam_info_pub.publish(tmp_caminfo)


if __name__ == '__main__':
    dd = DetectionDistance()
    try:
        dd.process()
    except rospy.ROSInitException:
        pass