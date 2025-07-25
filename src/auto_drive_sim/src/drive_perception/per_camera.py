#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 
import numpy as np
from std_msgs.msg import Float64

class PerCamera:
    def __init__(self):
        # ROS 노드 초기화
        print(f"PerCamera start")
        
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.view_BEV_CB)
        self.nwindows = 10
        self.img = CompressedImage()
        self.bridge = CvBridge()
        
    def view_BEV_CB(self,msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        self.img_y, self.img_x = img.shape[0:2]
        self.window_height = np.int(self.img_y / self.nwindows)
        filtered_img = self.detect_color(img,img_hsv)
        warped_img = self.BEV_img_warp(filtered_img,self.img_y,self.img_x)
        bin_img = self.img_binary(warped_img)
        # lane_detect = self.window_search(bin_img)
        # # 이미지를 확인하는 용도
        # cv2.imwrite("img.png", img)
        # cv2.imshow("filtered_img",filtered_img)
        # cv2.imshow("warped_img",warped_img)
        # cv2.imshow("bin_img",bin_img)
        # cv2.imshow("lane_detect",lane_detect)
        # cv2.waitKey(1)
        
    def detect_color(self,img,img_hsv):
        yellow_lower = np.array([15,128,0])
        yellow_upper = np.array([40,255,255])
        yellow_range = cv2.inRange(img_hsv,yellow_lower,yellow_upper)
        white_lower = np.array([0,0,192])
        white_upper = np.array([179,64,255])
        white_range = cv2.inRange(img_hsv,white_lower,white_upper)
        combined_range = cv2.bitwise_or(yellow_range,white_range)
        filtered_img = cv2.bitwise_and(img,img,mask=combined_range)
        # cv2.imwrite("filtered_img.png", filtered_img) # 저장하기 위한 코드
        return filtered_img
    
    def BEV_img_warp(self,filtered_img,y,x):
        src_point1 = [0,420]
        src_point2 = [275,260]
        src_point3 = [x - 275,260]
        src_point4 = [x,420]
        src_points = np.float32([src_point1,src_point2,src_point3,src_point4])
        
        dst_point1 = [x//8,480]
        dst_point2 = [x//8,0]
        dst_point3 = [x//8*7,0]
        dst_point4 = [x//8*7,480]
        dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])
        matrix = cv2.getPerspectiveTransform(src_points,dst_points)
        warped_img = cv2.warpPerspective(filtered_img,matrix,[x,y])
        # cv2.imwrite("warped_img.png", warped_img) # 저장하기 위한 코드
        return warped_img

    def img_binary(self,warped_img):
        grayed_img = cv2.cvtColor(warped_img,cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img>50] = 255
        # cv2.imwrite("bin_img.png", bin_img) # 저장하기 위한 코드 
        return bin_img

    def move_call(self):
        pass
