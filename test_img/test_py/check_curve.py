#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
import cv2
import numpy as np
import time

def img_shows(img,lines):
    # 결과 이미지 복사
    line_img = img.copy()

    # 라인 그리기
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 초록색 선

    # 출력
    cv2.imshow("Detected Lines", line_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def detect_color(img, img_hsv):
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
def BEV_img_warp(filtered_img,y,x):
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

def img_binary(warped_img):
    grayed_img = cv2.cvtColor(warped_img,cv2.COLOR_BGR2GRAY)
    bin_img = np.zeros_like(grayed_img)
    bin_img[grayed_img>50] = 255
    # cv2.imwrite("bin_img.png", bin_img) # 저장하기 위한 코드 
    return bin_img

class sliding:
    def __init__(self):
        self.midpoint = None
        self.leftx_base = None
        self.rightx_base = None

        self.window_height = None
        self.nonzero = None
        self.nonzeroy = None
        self.nonzerox = None
        self.left_blocked_flag = 0
        self.right_blocked_flag = 0
        
    def sliding_window_lane_calculation(self,x_current,y_current,lane_inds,prev_margin,x_prev,blocked_flag):
        flag = blocked_flag
        win_y_low = y_current - self.window_height // 2
        win_y_high = y_current + self.window_height // 2
        win_x_low = x_current - self.margin - prev_margin
        win_x_high = x_current + self.margin + prev_margin
        cv2.rectangle(self.out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0,255,0), 2)
        good_inds = ((self.nonzeroy >= win_y_low) & (self.nonzeroy < win_y_high) &
                            (self.nonzerox >= win_x_low) & (self.nonzerox < win_x_high)).nonzero()[0]
        lane_inds.append(good_inds)
        if len(good_inds) > self.minpix:
            mean_x = np.mean(self.nonzerox[good_inds])
            min_x = np.min(self.nonzerox[good_inds])
            max_x = np.max(self.nonzerox[good_inds])
            
            alpha = 1
            delta = mean_x + (mean_x - x_prev)
            x_current = int(alpha * mean_x + (1 - alpha) * delta)
            # x_current = int(mean_x )
            y_current -= self.window_height
            prev_margin = (max_x - min_x) // 4
            x_prev = x_current
            cv2.circle(self.out_img, (x_current, y_current), radius=5, color=(0, 0, 255), thickness=-1)
            flag = 0
        else:
            y_current -= self.window_height
            flag += 1
        return x_current,y_current, flag, prev_margin, x_prev
    
    def sliding_window_adaptive(self,binary_img, nwindows=15, margin=100, minpix=100):
        if len(binary_img.shape) == 2:
            self.out_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        else:
            self.out_img = binary_img.copy()
        # self.out_img = np.dstack([binary_img]*3) * 255
        histogram = np.sum(binary_img[binary_img.shape[0]//2:, :], axis=0)

        self.margin = margin
        prev_left_margin = margin
        prev_right_margin = margin
        self.minpix = minpix
        self.midpoint = histogram.shape[0] // 2
        self.leftx_base = np.argmax(histogram[:self.midpoint])
        self.rightx_base = np.argmax(histogram[self.midpoint:]) + self.midpoint

        self.window_height = binary_img.shape[0] // nwindows
        self.nonzero = binary_img.nonzero()
        self.nonzeroy = np.array(self.nonzero[0])
        self.nonzerox = np.array(self.nonzero[1])

        leftx_current = self.leftx_base
        leftx_prev = self.leftx_base
        lefty_current = binary_img.shape[0] - self.window_height // 2

        rightx_current = self.rightx_base
        rightx_prev = self.rightx_base
        righty_current = binary_img.shape[0] - self.window_height // 2

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            # 윈도우 범위 (적응형 이동)
            if self.left_blocked_flag < 3:
                leftx_current,lefty_current,self.left_blocked_flag, prev_left_margin, leftx_prev = self.sliding_window_lane_calculation(leftx_current,lefty_current,
                                                                                                               left_lane_inds,prev_left_margin,leftx_prev,self.left_blocked_flag)
            if self.right_blocked_flag < 3:
                rightx_current,righty_current,self.right_blocked_flag, prev_right_margin, rightx_prev  = self.sliding_window_lane_calculation(rightx_current,righty_current,
                                                                                                right_lane_inds,prev_right_margin,rightx_prev,self.right_blocked_flag)


            # print(f"Window {window}: lefty_current={lefty_current}, righty_current={righty_current}")
        # # 모든 인덱스 합치기
        # left_lane_inds = np.concatenate(left_lane_inds)
        # right_lane_inds = np.concatenate(right_lane_inds)

        # # 좌우 픽셀 좌표
        # leftx = self.nonzerox[left_lane_inds]
        # lefty = self.nonzeroy[left_lane_inds]
        # rightx = self.nonzerox[right_lane_inds]
        # righty = self.nonzeroy[right_lane_inds]
        # print(leftx.shape,lefty,rightx,righty)
        # # 폴리 피팅 (없으면 None)
        # left_fit = np.polyfit(lefty, leftx, 2) if len(leftx) > 0 else None
        # right_fit = np.polyfit(righty, rightx, 2) if len(rightx) > 0 else None

        # return left_fit, right_fit, self.out_img
        return self.out_img
_start = time.time()

path = './test_img/curve/img.png'
# path = './test_img/curve_2/img.png'
# path = './test_img/curve_3/img.png'
# path = './test_img/curve_4/img.png'
# path = './test_img/curve_5/img.png'
# path = './test_img/curve_6/img.png'
# path = './test_img/curve_301/img.png'
# path = './test_img/curve_302/img.png'
# bin_img = cv2.imread(path)
img = cv2.imread(path)
img_y, img_x = img.shape[0:2]
img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
filtered_img = detect_color(img,img_hsv)
warped_img = BEV_img_warp(filtered_img,img_y,img_x)
bin_img = img_binary(warped_img)
ss = sliding()
# left_fit, right_fit, vis_img = ss.sliding_window_adaptive(bin_img)
vis_img = ss.sliding_window_adaptive(bin_img)
elapsed = time.time() - _start
print(f"[Timer] 경과 시간: {elapsed:.6f}초")
# if left_fit is not None:
#     print("왼쪽 차선 곡선:", left_fit)
# if right_fit is not None:
#     print("오른쪽 차선 곡선:", right_fit)

cv2.imshow("Window Result", vis_img)
cv2.waitKey(0)