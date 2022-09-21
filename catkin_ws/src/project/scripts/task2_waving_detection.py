#!/usr/bin/env python3

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
from project.srv import WavingLeftRight
from project.srv import WavingLeftRightResponse


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



class WavingDetector:
    def __init__(self):
        rospy.init_node('waving_detector', anonymous=True)

        self.bridge = CvBridge()

        self.wave_detector = KeypointRCNN()

        # Subscriber
        rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)

        # publisher
        self.detection_result_pub = rospy.Publisher('/kp_image', Image, queue_size=10)

        # service
        self.wave_detection_service = rospy.Service('/wave_detection', WavingLeftRight, self.wave_detection)

        self.left_cnt = 0
        self.right_cnt = 0
        self.rgb_image = None

    def rgb_callback(self, data):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)




    def left_or_right_detection(self):

        if self.rgb_image is None:
            return

        rgb_img_copy= self.rgb_image.copy()
        tmp_rgb_image = copy.copy(self.rgb_image)

        dets = self.wave_detector.predict(PIL.Image.fromarray(rgb_img_copy))

        if len(dets) != 2:
            print('not two')
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

        #examine whitch person is waving (step5)

        hand_up_or_down = {"left_person":{"left_wrist":None,'right_wrist': None, 'left_elbow':None, 'right_elbow':None},\
            "right_person":{"left_wrist":None,'right_wrist': None, 'left_elbow':None, 'right_elbow':None}}

        for idx, k in enumerate(people['left_person']['key_points']):
            if KeypointRCNN.PART_STR[idx] in ['left_wrist', 'right_wrist', 'left_elbow', 'right_elbow']:
                hand_up_or_down['left_person'][KeypointRCNN.PART_STR[idx]] = k[:2]

        for idx, k in enumerate(people['right_person']['key_points']):
            if KeypointRCNN.PART_STR[idx] in ['left_wrist', 'right_wrist', 'left_elbow', 'right_elbow']:
                hand_up_or_down['right_person'][KeypointRCNN.PART_STR[idx]] = k[:2]

        elbow_wrist = {"left_person":None,'right_person':None }
        elbow_wrist['left_person'] = max((hand_up_or_down['left_person']['left_elbow'][1] - hand_up_or_down['left_person']['left_wrist'][1]), \
            (hand_up_or_down['left_person']['right_elbow'][1] - hand_up_or_down['left_person']['right_wrist'][1]))
        elbow_wrist['right_person'] = max((hand_up_or_down['right_person']['left_elbow'][1] - hand_up_or_down['right_person']['left_wrist'][1]), \
            (hand_up_or_down['right_person']['right_elbow'][1] - hand_up_or_down['right_person']['right_wrist'][1]))

        if elbow_wrist['left_person']>elbow_wrist['right_person']:
            self.left_cnt += 1
        else:
            self.right_cnt += 1

        tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['left_person']['left_elbow'][0]), int(hand_up_or_down['left_person']['left_elbow'][1])), 15, (0, 255, 0), thickness=-1)
        tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['left_person']['left_wrist'][0]), int(hand_up_or_down['left_person']['left_wrist'][1])), 15, (255, 0, 0), thickness=-1)
        tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['left_person']['right_elbow'][0]), int(hand_up_or_down['left_person']['right_elbow'][1])), 15, (100, 100, 0), thickness=-1)
        tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['left_person']['right_wrist'][0]), int(hand_up_or_down['left_person']['right_wrist'][1])), 15, (0, 100, 100), thickness=-1)
        tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['right_person']['left_elbow'][0]), int(hand_up_or_down['right_person']['left_elbow'][1])), 15, (0, 0, 255), thickness=-1)
        tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['right_person']['left_wrist'][0]), int(hand_up_or_down['right_person']['left_wrist'][1])), 15, (255, 255, 255), thickness=-1)
        tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['right_person']['right_elbow'][0]), int(hand_up_or_down['right_person']['right_elbow'][1])), 15, (100, 0, 100), thickness=-1)
        tmp_rgb_image = cv2.circle(tmp_rgb_image, (int(hand_up_or_down['right_person']['right_wrist'][0]), int(hand_up_or_down['right_person']['right_wrist'][1])), 15, (0, 0, 0), thickness=-1)
        tmp_rgb_image = cv2.cvtColor(tmp_rgb_image, cv2.COLOR_RGB2BGR)
        detection_result = self.bridge.cv2_to_imgmsg(tmp_rgb_image, "bgr8")
        self.detection_result_pub.publish(detection_result)



    def wave_detection(self, req):

        res = WavingLeftRightResponse()

        self.left_cnt = 0
        self.right_cnt = 0

        t = rospy.Time.now()

        while rospy.Time.now().secs - t.secs < 20:
            rospy.sleep(0.05)
            self.left_or_right_detection()


        if self.left_cnt > self.right_cnt:
            res.left_or_right = 'left'
            l = self.left_cnt
            r = self.right_cnt
            rospy.loginfo("left" + str(l) + "right" + str(r))
            rospy.loginfo("left person is waving a hand")
        else:
            res.left_or_right = 'right'
            l = self.left_cnt
            r = self.right_cnt
            rospy.loginfo("left" + str(l) + "right" + str(r))
            rospy.loginfo("right person is waving a hand")
        print(res)
        return res


if __name__ == '__main__':
    wave_detector = WavingDetector()
    rospy.spin()


