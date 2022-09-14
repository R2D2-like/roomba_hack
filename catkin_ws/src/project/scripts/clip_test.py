#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from project.msg import ImageArray
from cv_bridge import CvBridge
from pytorchyolo import detect, models
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import cv2
import copy

import torch
import clip
import PIL

class Detection_neo:
    def __init__(self):
        rospy.init_node('Detection_neo', anonymous=True)

        self.detect_result=[]

        self.device='cpu'
        self.model, self.preprocess = clip.load("ViT-L/14@336px", device='cpu', jit=False)
        rgb_sub=rospy.Subscriber("/task1/mask_result_for_clip",ImageArray,self.callback)
        self.bridge=CvBridge()
        self.rgb_image,self.depth_image,self.camera_info=None,None,None


    def callback(self,data1):
        for i in len(data1):
            cv_array=self.bridge.imgmsg_to_cv2(data1[i],'bgr8')
            cv_array=cv2.cvtColor(cv_array,cv2.COLOR_BGR2RGB)
            self.rgb_image=PIL.Image.fromarray(np.array(cv_array))
            print(type(self.rgb_image))
            dn.process()


    def process(self):
        self.image=self.preprocess(self.rgb_image).unsqueeze(0).to(self.device)
        texts=["a strawberry","a sports ball","an apple","a banana","a toy plane","a chips can","a rubiks cube","a yellow wood block"]
        texts2=["a photo of a strawberry, a type of fruit","a photo of a sports ball","a photo of an apple, type of fruit","a photo of a banana, type of fruit","a phot of a toy plane","a photo of a chips can","a photo of a rubiks cube","a photo of a yellow wood block"]
        text=clip.tokenize(texts2).to(self.device)

        with torch.no_grad():
            image_features=self.model.encode_image(self.image)
            text_features=self.model.encode_text(text)

            logits_per_image,logits_per_text=self.model(self.image, text)
            probs=logits_per_image.softmax(dim=-1).cpu().numpy()
        idx=np.argmax(probs)
        if texts[idx] in self.detect_result:
            pass
        else:
            self.detect_result.append(texts[idx])


        print("Label probs",probs)
        print(self.detect_result)





if __name__ =="__main__":
    dn=Detection_neo()
    rospy.spin()