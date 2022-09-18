#!/usr/bin/env python3

from os import TMP_MAX
from re import X
from subprocess import call
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



# from project.msg import ImageArray

import torch
import clip
import PIL

class DetectionDistance:
    def __init__(self):
        rospy.init_node('task1_clip_unified', anonymous=True)

        # Publisher
        #self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)
        #self.depth_mask_pub = rospy.Publisher('/task1/masked_depth/image', Image, queue_size=10)
        #self.cam_info_pub = rospy.Publisher('/task1/masked_depth/camera_info', CameraInfo, queue_size=10)
        self.detect_result=[]
        self.texts=["a strawberry","a sports ball","an apple","a banana","a toy plane","a chips can","a rubiks cube","a yellow wood block"]
        self.texts2=["a photo of a strawberry, a type of fruit","a photo of a blue sports ball","a photo of an apple, a type of fruit","a photo of a yellow banana, type of fruit","a phot of a toy plane","a photo of a chips can ","a photo of a colorful rubiks cube","a photo of a yellow cube"]

        self.device='cuda'
        self.model, self.preprocess = clip.load("ViT-L/14@336px", device='cuda', jit=False)
        self.counter = [0,0,0,0,0,0,0,0]
        self.text=clip.tokenize(self.texts2).to(self.device)

        # Subscriber
        rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image,self.callback_rgb)

        

        self.bridge = CvBridge()
        self.rgb_image = None

        #黄色
        self.hsv_min_Y = np.array([20, 80, 10])    # 抽出する色の下限(HSV)
        self.hsv_max_Y = np.array([50, 255, 255])    # 抽出する色の上限(HSV)

        # 赤色のHSVの値域1
        self.hsv_min_R1 = np.array([0,64,0])
        self.hsv_max_R1 = np.array([30,255,255])

        # 赤色のHSVの値域2
        self.hsv_min_R2 = np.array([150,64,0])
        self.hsv_max_R2 = np.array([179,255,255])

        # 青色のHSVの値域
        self.hsv_min_B = np.array([90, 64, 0])
        self.hsv_max_B = np.array([150,255,255])

        # 緑色のHSVの値域1
        self.hsv_min_G = np.array([30, 64, 0])
        self.hsv_max_G = np.array([90,255,255])

    def callback_rgb(self, data):
        cv_array = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.bgr_image = cv_array
        self.hsv_image = cv2.cvtColor(cv_array, cv2.COLOR_BGR2HSV) # 画像をHSVに変換
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

    def process(self):
        # path = "/root/roomba_hack/catkin_ws/src/three-dimensions_tutorial/yolov3/"

        # # load category
        # with open(path+"data/coco.names") as f:
        #     category = f.read().splitlines()

        # # prepare model
        # model = models.load_model(path+"config/yolov3.cfg", path+"weights/yolov3.weights")

        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue

            # inference

            tmp_bgr_image = copy.copy(self.bgr_image)
            tmp_hsv_image = copy.copy(self.hsv_image)
            tmp_rgb_image = copy.copy(self.rgb_image)
            #tmp_depth = copy.copy(self.depth_image)
            # tmp_caminfo = copy.copy(self.cam_info)
            # print(tmp_caminfo.header)


            rec = np.zeros_like(tmp_bgr_image)
            cv2.rectangle(rec, (0,int(rec.shape[0]*2/5)), (int(rec.shape[1]),int(rec.shape[0])), (255, 255, 255), -1)

            tmp_bgr_image = cv2.bitwise_and(tmp_bgr_image, rec)
            tmp_hsv_image = cv2.bitwise_and(tmp_hsv_image, rec)
            tmp_rgb_image = cv2.bitwise_and(tmp_rgb_image, rec)

            maskY = cv2.inRange(tmp_hsv_image, self.hsv_min_Y, self.hsv_max_Y)
            maskR1 = cv2.inRange(tmp_hsv_image, self.hsv_min_R1, self.hsv_max_R1)
            maskR2 = cv2.inRange(tmp_hsv_image, self.hsv_min_R2, self.hsv_max_R2)
            maskB = cv2.inRange(tmp_hsv_image, self.hsv_min_B, self.hsv_max_B)
            maskG = cv2.inRange(tmp_hsv_image, self.hsv_min_G, self.hsv_max_G)

            maskRGBY = maskY + maskR1 + maskR2 + maskB + maskG
            mask_result = cv2.bitwise_and(tmp_bgr_image, tmp_bgr_image, mask=maskRGBY) # mask_result is BGR
            # グレースケールに変換する
            gray = cv2.cvtColor(mask_result,cv2.COLOR_BGR2GRAY)
            # 2値化する
            ret, bin_img = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
            # 輪郭を抽出する。
            contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours2 = list(filter(lambda x: cv2.contourArea(x) >= 1000, contours))

            # crop_image_list = []
            # roslist = ImageArray()
            for i, cnt in enumerate(contours2):
            # 輪郭に外接する長方形を取得する。
                x, y, width, height = cv2.boundingRect(cnt)
                mask = np.zeros_like(maskRGBY)
                mask_y1 = int(y-height/5)
                mask_y2 = int(y+height*6/5)
                mask_x1 = int(x-width/5)
                mask_x2 = int(x+width*6/5)
                
                if not(0 < = mask_y1 <= 720):
                    mask_y1 = y
                if not(0 < = mask_y2 <= 720):
                    mask_y2 = y+height
                if not(0 < = mask_x1 <= 1280):
                    mask_x1 = x
                if not(0 < = mask_x2 <= 1280):
                    mask_x2 = x+width

                #mask[int(y-height/5):int(y+height*6/5),int(x-width/5):int(x+width*6/5)] = 1
                mask[mask_y1:mask_y2, mask_x1:mask_x2] = 1
                result = cv2.bitwise_and(tmp_rgb_image, tmp_rgb_image, mask=mask) #RGB

                image_pil = Image.fromarray(result)   #opencv2pil
                image_pil = image_pil.convert('RGB')
                self.image=self.preprocess(image_pil).unsqueeze(0).to(self.device)
                
                with torch.no_grad():
                    image_features=self.model.encode_image(self.image)
                    text_features=self.model.encode_text(self.text)

                    logits_per_image,logits_per_text=self.model(self.image, text)
                    probs=logits_per_image.softmax(dim=-1).cpu().numpy()

                idx=np.argmax(probs)
                area = height*width


                #画像処理による修正


                #sports ball

                if idx == 1: 

                #青色抽出
                extractB = cv2.bitwise_and(result, result, mask=maskB)
                gray = cv2.cvtColor(extractB,cv2.COLOR_RGB2GRAY)
                ret, bin_img = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)

                # 輪郭を抽出する。
                contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours2 = list(filter(lambda x: cv2.contourArea(x) >= 80, contours))
                target_idx = 0
                # 輪郭に外接する長方形を取得する。
                
                if len(contours2) == 0: #青がない
    
                    #緑抽出
                    extractG = cv2.bitwise_and(result, result, mask=maskG)
                    gray = cv2.cvtColor(extractG,cv2.COLOR_RGB2GRAY)
                    ret, bin_img = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
                    contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    contours2 = list(filter(lambda x: cv2.contourArea(x) >= 80, contours))

                    # 輪郭に外接する長方形を取得する。

                    if len(contours2) != 0: #緑がない
                        idx = 0
                        #print('strawberry')
                    else:
                        probs = list(probs)
                        if probs[0][0] > probs[0][2]:
                            idx = 0
                            #print('strawberry')
                        else:
                            idx = 2
                            #print('apple')
        
                else: #青がある
                    x2, y2, width2, height2 = cv2.boundingRect(contours2[target_idx])
                    if height2*width2/area > 0.6:
                        idx = 1
                        #print('soccer ball')
                    else:
                        #緑抽出
                        extractG = cv2.bitwise_and(result, result, mask=maskG)
                        gray = cv2.cvtColor(extractG,cv2.COLOR_RGB2GRAY)
                        ret, bin_img = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
                        contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        contours2 = list(filter(lambda x: cv2.contourArea(x) >= 80, contours))

                        # 輪郭に外接する長方形を取得する。

                        if len(contours2) != 0: #緑がある
                            idx = 0
                            #print('strawberry')
                        else: #緑がない
                            probs = list(probs)
                            if probs[0][0] > probs[0][2]:
                                idx = 0
                                #print('strawberry')
                            else:
                                idx = 2
                                #print('apple')

                # apple

                if idx == 2:
                    if abs(width-height)/max(width, height) > 0.2:
                        idx = 5
                        #print('chips')
                    else:
                        #黄色抽出
                        extractY = cv2.bitwise_and(result, result, mask=maskY)
                        gray = cv2.cvtColor(extractY,cv2.COLOR_RGB2GRAY)
                        ret, bin_img = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
                        contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        contours2 = list(filter(lambda x: cv2.contourArea(x) >= 80, contours))
                        target_idx = 0
  
                        if len(contours2) == 0: #黄色がない
                            idx = 2
                            #print('apple')
                        else:
                            if width/height >1.5:
                                idx = 3
                                #print('banana')
                            else:
                                idx = 7
                                #print('yellow block')

                #strawberry or apple

                if idx == 0 or idx == 2:

                    #緑抽出
                    extractG = cv2.bitwise_and(result, result, mask=maskG)
                    gray = cv2.cvtColor(extractG,cv2.COLOR_RGB2GRAY)
                    ret, bin_img = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
                    contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    contours2 = list(filter(lambda x: cv2.contourArea(x) >= 80, contours))

                    # 輪郭に外接する長方形を取得する。

                    if len(contours2) != 0: #緑がある
                        idx = 0
                        #print('strawberry')
                    else: #緑がない
                        probs = list(probs)
                        if probs[0][0] > probs[0][2]:
                            idx = 0
                            #print('strawberry')
                        else:
                            idx = 2
                            #print('apple')

                # chips

                if idx == 5:

                    #黄色抽出
                    extractY = cv2.bitwise_and(result, result, mask=maskY)
                    gray = cv2.cvtColor(extractY,cv2.COLOR_RGB2GRAY)
                    ret, bin_img = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
                    contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    contours2 = list(filter(lambda x: cv2.contourArea(x) >= 80, contours))
                    target_idx = 0
                    # 輪郭に外接する長方形を取得する。

                    if len(contours2) == 0: #黄色がない
                        idx = 5
                        #print('chips')
                    else:
                        x2, y2, width2, height2 = cv2.boundingRect(contours2[target_idx])
                        if height2*width2/area < 0.2:
                            idx = 5
                            #print('chips')
                        else:
                            if width/height >1.5:
                                idx = 3
                                #print('banana')
                            else:
                                idx = 7
                                #print('yellow block')

                #yellow_block

                if idx == 7:
                    #黄色抽出
                    extractY = cv2.bitwise_and(result, result, mask=maskY)
                    gray = cv2.cvtColor(extractY,cv2.COLOR_RGB2GRAY)
                    ret, bin_img = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)

                    # 輪郭を抽出する。
                    contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    contours2 = list(filter(lambda x: cv2.contourArea(x) >= 80, contours))
                    target_idx = 0
                    # 輪郭に外接する長方形を取得する。
                    if len(contours2) == 0: #黄色がない
                        idx = 6
                        #print('rubiks')
                    else:
                        x2, y2, width2, height2 = cv2.boundingRect(contours2[target_idx])
                        if height2*width2/area < 0.6:
                            idx = 6
                            #print('rubiks')
                        else:
                            if width/height >1.5:
                                idx = 3
                                #print('banana')
                            else:
                                idx = 7
                                #print('yellow block')



                self.counter[idx] += 1

                print("Label probs" + str(probs))
                rint("a strawberry" + str(self.counter[0]) + "\n a sports ball"+ str(self.counter[1]) + "\n an apple"+ str(self.counter[2])+\
                "\n a banana"+ str(self.counter[3])+"\n a toy plane"+ str(self.counter[4])+"\n a chips can"+ str(self.counter[5])+"\n a rubiks cube"+\
                str(self.counter[6])+"\n a yellow wood block"+ str(self.counter[7]))

                

                # crop_image_list.append(result)
                #self.mask_result_pub.publish(result)
                #rospy.sleep(0.1)
            # print(len(crop_image_list))
            # roslist.image_array = crop_image_list
            # self.mask_result_pub.publish(roslist)

            # boxes = detect.detect_image(model, tmp_rgb_image)
            # # [[x1, y1, x2, y2, confidence, class]]
            # # cv_array2 = self.bridge.imgmsg_to_cv2(tmp_depth, '32FC1')
            # #mask = np.zeros_like(tmp_depth)

            # # plot bouding box
            # for box in boxes:
            #     x1, y1, x2, y2 = map(int, box[:4])
            #     cls_pred = int(box[5])
            #     #mask[y1:y2,x1:x2] = 1

            #     #print(mask)

            #     tmp_rgb_image = cv2.rectangle(tmp_rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
            #     tmp_rgb_image = cv2.putText(tmp_rgb_image, category[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            #     cx, cy = (x1+x2)//2, (y1+y2)//2
            #     print(category[cls_pred], self.depth_image[cy][cx]/1000, "m")

            # # publish image

            # tmp_image = cv2.cvtColor(tmp_rgb_image, cv2.COLOR_RGB2BGR)
            # detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            # mask_result = np.where(maskRGBY,tmp_depth,0)
            # mask_result = self.bridge.cv2_to_imgmsg(mask_result, "passthrough")
            # mask_result.header = tmp_caminfo.header
            # self.detection_result_pub.publish(detection_result)
            # self.depth_mask_pub.publish(mask_result)
            # self.cam_info_pub.publish(tmp_caminfo)


if __name__ == '__main__':
    dd = DetectionDistance()
    try:
        dd.process()
    except rospy.ROSInitException:
        pass

