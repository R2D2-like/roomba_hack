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

        self.device='cuda'
        self.model, self.preprocess = clip.load("ViT-L/14@336px", device='cuda', jit=False)
        rgb_sub=rospy.Subscriber("/task1/mask_result_for_clip",ImageArray,self.callback)
        self.bridge=CvBridge()
        self.rgb_image,self.depth_image,self.camera_info=None,None,None

        #self.threshold={"a strawberry":0.2,"a sports ball":0.2,"an apple":0.2,"a banana":0.2,"a toy plane":0.2,"a chips can":0.2,"a rubiks cube":0.2,"a yellow wood block":0.2}
        self.counter = [0,0,0,0,0,0,0,0]

    def callback(self,data1):
        for i in range(len(data1.image_array)):
            cv_array=self.bridge.imgmsg_to_cv2(data1.image_array[i],'bgr8')
            cv_array=cv2.cvtColor(cv_array,cv2.COLOR_BGR2RGB)
            self.rgb_image=PIL.Image.fromarray(np.array(cv_array))
            print(type(self.rgb_image))
            dn.process()


    def process(self):
        self.image=self.preprocess(self.rgb_image).unsqueeze(0).to(self.device)
        texts=["a strawberry","a sports ball","an apple","a banana","a toy plane","a chips can","a rubiks cube","a yellow wood block"]
        texts2=["a photo of a strawberry, a type of fruit","a photo of a blue sports ball","a photo of an apple, a type of fruit","a photo of a yellow banana, type of fruit","a phot of a toy plane","a photo of a chips can ","a photo of a colorful rubiks cube","a photo of a yellow cube"]
        text=clip.tokenize(texts2).to(self.device)

        with torch.no_grad():
            image_features=self.model.encode_image(self.image)
            text_features=self.model.encode_text(text)

            logits_per_image,logits_per_text=self.model(self.image, text)
            probs=logits_per_image.softmax(dim=-1).cpu().numpy()



        idx=np.argmax(probs)


        '''
        if texts[idx] in self.detect_result:
            pass
        else:
            self.detect_result.append(texts[idx])
        '''

        #probs=list(probs)
        self.counter[idx] += 1

        '''
        for idx in range(len(probs)):
            if texts[idx] in self.detect_result:
                continue
            elif probs[idx] >= self.threshold[texts[idx]]:
                self.detect_result.append(texts[idx])
        '''


        print("Label probs" + str(probs))
        print("a strawberry" + str(self.counter[0]) + "\n a sports ball"+ str(self.counter[1]) + "\n an apple"+ str(self.counter[2])+\
            "\n a banana"+ str(self.counter[3])+"\n a toy plane"+ str(self.counter[4])+"\n a chips can"+ str(self.counter[5])+"\n a rubiks cube"+\
                str(self.counter[6])+"\n a yellow wood block"+ str(self.counter[7]))




if __name__ =="__main__":
    dn=Detection_neo()
    rospy.spin()
