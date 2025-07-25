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
path = './test_img/curve/img.png'
# path = './test_img/curve_3/img.png'
# bin_img = cv2.imread(path)
img = cv2.imread(path)
img_y, img_x = img.shape[0:2]
img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
filtered_img = detect_color(img,img_hsv)
warped_img = BEV_img_warp(filtered_img,img_y,img_x)
bin_img = img_binary(warped_img)


# # 출력
# cv2.imshow("bin_img", bin_img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
def sliding_window_adaptive(binary_img, nwindows=10, margin=80, minpix=10):
    out_img = np.dstack([binary_img]*3) * 255
    histogram = np.sum(binary_img[binary_img.shape[0]//2:, :], axis=0)

    midpoint = histogram.shape[0] // 2
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    window_height = binary_img.shape[0] // nwindows
    nonzero = binary_img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    leftx_current = leftx_base
    lefty_current = binary_img.shape[0] - window_height // 2

    rightx_current = rightx_base
    righty_current = binary_img.shape[0] - window_height // 2

    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        # 윈도우 범위 (적응형 이동)
        win_y_low_left = lefty_current - window_height // 2
        win_y_high_left = lefty_current + window_height // 2
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin

        win_y_low_right = righty_current - window_height // 2
        win_y_high_right = righty_current + window_height // 2
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # 시각화용 사각형
        cv2.rectangle(out_img, (win_xleft_low, win_y_low_left), (win_xleft_high, win_y_high_left), (0,255,0), 2)
        cv2.rectangle(out_img, (win_xright_low, win_y_low_right), (win_xright_high, win_y_high_right), (0,255,0), 2)

        # 윈도우 안 픽셀 인덱스
        good_left_inds = ((nonzeroy >= win_y_low_left) & (nonzeroy < win_y_high_left) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low_right) & (nonzeroy < win_y_high_right) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
            lefty_current = int(np.min(nonzeroy[good_left_inds]))
        else:
            lefty_current = max(lefty_current - window_height, 0)

        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))
            righty_current = int(np.min(nonzeroy[good_right_inds]))
        else:
            righty_current = max(righty_current - window_height, 0)

        print(f"Window {window}: lefty_current={lefty_current}, righty_current={righty_current}")

    # 모든 인덱스 합치기
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # 좌우 픽셀 좌표
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # 폴리 피팅 (없으면 None)
    left_fit = np.polyfit(lefty, leftx, 2) if len(leftx) > 0 else None
    right_fit = np.polyfit(righty, rightx, 2) if len(rightx) > 0 else None

    return left_fit, right_fit, out_img

left_fit, right_fit, vis_img = sliding_window_adaptive(bin_img)

if left_fit is not None:
    print("왼쪽 차선 곡선:", left_fit)
if right_fit is not None:
    print("오른쪽 차선 곡선:", right_fit)

cv2.imshow("bin_img", bin_img)
cv2.imshow("Window Result", vis_img)
cv2.waitKey(0)