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

def out_rotary_Img__crop_center_vertical(image, h=50):
    """
    이미지의 세로 중앙을 기준으로 위아래로 h만큼 자른 이미지를 반환합니다.

    Parameters:
        image (numpy.ndarray): 입력 이미지
        h (int): 중앙 기준 위아래로 자를 픽셀 수 (총 높이는 2*h)

    Returns:
        cropped_image (numpy.ndarray): 잘린 이미지
    """
    height = image.shape[0]
    start = max(0, height // 2 - h)
    end = min(height, height // 2 + h)
    return image[start:end, :]
    
def out_rotary_calculate_slope(x1, y1, x2, y2):
    if x2 - x1 == 0:
        return float('inf')  # 수직선 처리
    return (y2 - y1) / (x2 - x1)

def out_rotary_check_positive_and_negative(slopes):
    has_pos = any(s > 0 and s != float('inf') for s in slopes)
    has_neg = any(s < 0 and s != float('inf') for s in slopes)
    return has_pos and has_neg

def out_rotary_is_rotary_exit(image):
    """
    누운 V자 형태로 출구가 보이는지 판단합니다.

    Parameters:
        image (numpy.ndarray): 입력 이미지
        h (int): 하단에서 위로 자를 ROI 높이

    Returns:
        bool: 출구 판단 결과 (True = 출구)
    """
    height, width = image.shape[:2]

    # 1. 전처리
    edges = cv2.Canny(image, 50, 150)

    # 2. 라인 추출
    lines = cv2.HoughLinesP(edges, 1, 2 * np.pi/180, threshold=50,
                            minLineLength=50, maxLineGap=40)
    if lines is None or len(lines) < 2:
        return False
    slopes = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = out_rotary_calculate_slope(x1, y1, x2, y2)
        slopes.append(slope)

    return out_rotary_check_positive_and_negative(slopes)

# 이미지 경로
img_path = './test_img/rotary_2/bin_img2.png'
binary = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
_start = time.time()
cropped = out_rotary_Img__crop_center_vertical(binary, 100)
elapsed = time.time() - _start
print(f"[Timer] 경과 시간: {elapsed:.6f}초")
_start = time.time()
print(out_rotary_is_rotary_exit(cropped))
elapsed = time.time() - _start
print(f"[Timer] 경과 시간: {elapsed:.6f}초")
