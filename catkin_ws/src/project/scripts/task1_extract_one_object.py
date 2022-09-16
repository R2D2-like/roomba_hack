#!/usr/bin/env python3

from os import TMP_MAX
from re import X
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

from project.msg import ImageArray

class DetectionDistance:
    def __init__(self):
        rospy.init_node('task1_extract_object', anonymous=True)

        # Publisher
        #self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)
        #self.depth_mask_pub = rospy.Publisher('/task1/masked_depth/image', Image, queue_size=10)
        #self.cam_info_pub = rospy.Publisher('/task1/masked_depth/camera_info', CameraInfo, queue_size=10)
        self.mask_result_pub = rospy.Publisher('/task1/mask_result_for_clip', ImageArray, queue_size=10)

        # Subscriber
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        cam_info_sub = message_filters.Subscriber('/camera/color/camera_info',CameraInfo)
        message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, cam_info_sub], 10, 1.0).registerCallback(self.callback_rgbd)

        self.bridge = CvBridge()
        self.rgb_image, self.depth_image = None, None

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

    def callback_rgbd(self, data1, data2, data3):
        cv_array = self.bridge.imgmsg_to_cv2(data1, 'bgr8')
        self.bgr_image = cv_array
        self.hsv_image = cv2.cvtColor(cv_array, cv2.COLOR_BGR2HSV) # 画像をHSVに変換
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

        cv_array = self.bridge.imgmsg_to_cv2(data2, 'passthrough')
        self.depth_image = cv_array

        # self.cam_info = data3

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

            crop_image_list = []
            roslist = ImageArray()
            for i, cnt in enumerate(contours2):
            # 輪郭に外接する長方形を取得する。
                x, y, width, height = cv2.boundingRect(cnt)
                mask = np.zeros_like(maskRGBY)
                mask[y:y+height,x:x+width] = 1
                result = cv2.bitwise_and(tmp_rgb_image, tmp_rgb_image, mask=mask) #RGB
                result = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
                result = self.bridge.cv2_to_imgmsg(result, "bgr8")
                crop_image_list.append(result)
                #self.mask_result_pub.publish(result)
                #rospy.sleep(0.1)
            print(len(crop_image_list))
            roslist.image_array = crop_image_list
            self.mask_result_pub.publish(roslist)

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

