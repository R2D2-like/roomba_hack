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
from std_msgs.msg import String


class KeypointRCNN:

    COCO_INSTANCE_CATEGORY_NAMES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
    ]

    PART_STR = ["nose","left_eye","right_eye","left_ear","right_ear","left_shoulder",
                "right_shoulder","left_elbow","right_elbow","left_wrist","right_wrist",
                "left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]
    
    def __init__(self):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        model = torchvision.models.detection.keypointrcnn_resnet50_fpn(pretrained=True)
        self.model = model.to(self.device)
        self.model.eval()

    def predict(self, rgb_img, threshold=0.6):
        transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
        img = transform(rgb_img).to(self.device)
        with torch.no_grad():
            pred = self.model([img])[0]
       
        #tensor2np.array2list       
        pred_class = [KeypointRCNN.COCO_INSTANCE_CATEGORY_NAMES[i] for i in list(pred['labels'].detach().cpu().numpy())] 
        pred_boxes = [[(i[0], i[1]), (i[2], i[3])] for i in list(pred['boxes'].detach().cpu().numpy())]
        pred_score = list(pred['scores'].detach().cpu().numpy())
        pred_keypoints = list(pred['keypoints'].cpu().detach().numpy())

        #list of idx, whose val is pass the threshold and category(step1)
        pred_pass = [idx for idx, val in enumerate(pred_score) if (val > threshold)and(pred_class[idx] == "person")]

        if len(pred_pass) <= 0:
            return []

        #cut the unnecessarry elements
        pred_boxes = pred_boxes[:pred_pass[-1]+1]
        pred_class = pred_class[:pred_pass[-1]+1]
        pred_keypoints = pred_keypoints[:pred_pass[-1]+1]

        #raw_reselt
        det_results = [{'box': box, 'label': l, 'key_points': kp} for box, l , kp  in zip(pred_boxes, pred_class, pred_keypoints)]
        #without_overlap_det_results(step2)
        without_overlap_det_results = []
        for det_result in det_results:
            flag = False
            if len(without_overlap_det_results) == 0:
                without_overlap_det_results.append(det_results[0])
                continue
            x0 = det_result["box"][0][0]
            y0 = det_result["box"][0][1]
            x1 = det_result["box"][1][0]
            y1 = det_result["box"][1][1]

            for without_overlap_det_result in without_overlap_det_results:
                (X0,Y0),(X1,Y1) = without_overlap_det_result["box"]
                if (X0 < x0) and (x0 < X1):#x0 is within the area of one of the without_overlap_det_result's bounding boxs, so we should not include det_result
                    flag = True
                    break
                if (x0 < X0) and (X0 < x1):#X0 is within the area of  the det_result's bounding boxs, so we should not include det_result
                    flag = True
                    break
            
            without_overlap_det_results.append(det_result)

        return without_overlap_det_results


    

class DetectWavingPersonAnkle:
    def __init__(self):
        rospy.init_node('detection_waving_person', anonymous=True)

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)
        self.depth_mask_pub = rospy.Publisher('/depth_mask', Image, queue_size=10)
        self.cam_info_pub = rospy.Publisher('/camerainfo/depth_mask', CameraInfo, queue_size=10)

        
        self.ankle_mask_result_pub = rospy.Publisher('/kp_image/ankle', Image, queue_size=10)

        # Subscriber
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        cam_info_sub = message_filters.Subscriber('/camera/color/camera_info',CameraInfo)
        message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, cam_info_sub], 10, 1.0).registerCallback(self.callback_rgbd)

        trigger = rospy.Subscriber('/mask/ankle/trigger', String, self.process) #main

        self.bridge = CvBridge()
        self.rgb_image, self.depth_image = None, None

        self.wave_detector = KeypointRCNN()


    def callback_rgbd(self, data1, data2, data3):
        print("called@callback_rgbd")
        cv_array = self.bridge.imgmsg_to_cv2(data1, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

        cv_array = self.bridge.imgmsg_to_cv2(data2, 'passthrough')
        self.depth_image = cv_array

        self.cam_info = data3


    def extract_ancle_point(self, LorR):
        
        rgb_img_copy= self.rgb_image.copy()
        tmp_rgb_image = copy.copy(self.rgb_image)
        

        dets = self.wave_detector.predict(PIL.Image.fromarray(rgb_img_copy))
        print('@extract_ancle_point')

        if len(dets) != 2:
            return

        #sort biggest->smallset (step3)
        for det in dets:
            box_area= (det["box"][1][0]-det["box"][0][0])*(det["box"][1][1]-det["box"][0][1])
            det.update({"box_area":box_area})

        #dets = sorted(dets.items(),key=lambda det: det["box_area"], reverse=True)
        dets.sort(key=lambda det: det["box_area"], reverse=True)
        
        # examine the biggest BB is whether left or right person (step4)

        people = {}
   

        if (dets[0]["box"][0][0]+dets[0]["box"][1][0])/2 < (dets[1]["box"][0][0]+dets[1]["box"][1][0])/2:
            people["left_person"] = dets[0]
            people["right_person"] = dets[1]
        else:
            people["left_person"] = dets[1]
            people["right_person"] = dets[0]

    

        hand_up_or_down = {"left_person":{"left_ankle":None}, "right_person":{"right_ankle":None}}

        if LorR == "left":
            for idx, k in enumerate(people['left_person']['key_points']):
                if KeypointRCNN.PART_STR[idx] in ["right_ankle"]:
                    hand_up_or_down['left_person'][KeypointRCNN.PART_STR[idx]] = k[:2]
                    x=int(hand_up_or_down['left_person'][KeypointRCNN.PART_STR[idx]][0])
                    y=int(hand_up_or_down['left_person'][KeypointRCNN.PART_STR[idx]][1])
                    tmp_rgb_image = cv2.circle(tmp_rgb_image, (x, y), 15, (0, 255, 0), thickness=-1)
        else:
            for idx, k in enumerate(people['right_person']['key_points']):
                if KeypointRCNN.PART_STR[idx] in ["left_ankle"]:
                    hand_up_or_down['right_person'][KeypointRCNN.PART_STR[idx]] = k[:2]       
                    x=int(hand_up_or_down['right_person'][KeypointRCNN.PART_STR[idx]][0])
                    y=int(hand_up_or_down['right_person'][KeypointRCNN.PART_STR[idx]][1])
                    tmp_rgb_image = cv2.circle(tmp_rgb_image, (x, y), 15, (255, 0, 0), thickness=-1)                                                                                                                                                           

        # tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['left_person']['left_elbow'][0]), int(hand_up_or_down['left_person']['left_elbow'][1])), 15, (0, 255, 0), thickness=-1)
        # tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['left_person']['left_wrist'][0]), int(hand_up_or_down['left_person']['left_wrist'][1])), 15, (255, 0, 0), thickness=-1)
        # tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['left_person']['right_elbow'][0]), int(hand_up_or_down['left_person']['right_elbow'][1])), 15, (100, 100, 0), thickness=-1)
        # tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['left_person']['right_wrist'][0]), int(hand_up_or_down['left_person']['right_wrist'][1])), 15, (0, 100, 100), thickness=-1)
        # tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['right_person']['left_elbow'][0]), int(hand_up_or_down['right_person']['left_elbow'][1])), 15, (0, 0, 255), thickness=-1)
        # tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['right_person']['left_wrist'][0]), int(hand_up_or_down['right_person']['left_wrist'][1])), 15, (255, 255, 255), thickness=-1)
        # tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['right_person']['right_elbow'][0]), int(hand_up_or_down['right_person']['right_elbow'][1])), 15, (100, 0, 100), thickness=-1)
        # tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['right_person']['right_wrist'][0]), int(hand_up_or_down['right_person']['right_wrist'][1])), 15, (0, 0, 0), thickness=-1)

        tmp_rgb_image = cv2.cvtColor(tmp_rgb_image, cv2.COLOR_RGB2BGR)
        detection_result_mask = self.bridge.cv2_to_imgmsg(tmp_rgb_image, "bgr8")
        self.ankle_mask_result_pub.publish(detection_result_mask)

        x_y_list = [x,y]
        print(x_y_list)

        return x_y_list



    def process(self,msg):
        print("called@process")
        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue
            
            LorR = msg.data
            print(msg)
            tmp_depth = copy.copy(self.depth_image)
            tmp_caminfo = copy.copy(self.cam_info)
            xy = self.extract_ancle_point(LorR)
            x = xy[0]
            y = xy[1]
            print("x : " + str(x) + ", y : " + str(y))

            mask = np.zeros_like(tmp_depth)
            print(mask.shape)
            mask[y-50:y+50,x-50:x+50] = 1
            
            # publish image
            mask_result = np.where(mask,tmp_depth,0)
            mask_result = self.bridge.cv2_to_imgmsg(mask_result, "passthrough")
            mask_result.header = tmp_caminfo.header
            self.depth_mask_pub.publish(mask_result)
            self.cam_info_pub.publish(tmp_caminfo)

# def main_action(trigger):
#     if trigger:
#         dwpk = DetectWavingPersonAnkle()
#         try:
#             dwpk.process()
#         except rospy.ROSInitException:
#             pass

# def trigger_flag(req):



if __name__ == '__main__':
    dwpk = DetectWavingPersonAnkle()
    rospy.spin()

   
