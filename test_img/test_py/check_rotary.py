#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
import cv2

# 이미지 경로
img_path = './test_img/rotary/stop_bin_img.png'

# # 이미지 읽기
# image = cv2.imread(img_path)

import cv2
import numpy as np

def enter_rotary_Img__crop_bottom(image, pixels=100):
    """
    이미지 아래쪽에서 지정한 픽셀 수만큼 잘라낸 이미지를 반환합니다.

    Parameters:
        image (numpy.ndarray): 입력 이미지
        pixels (int): 아래쪽에서 잘라낼 픽셀 수

    Returns:
        cropped_image (numpy.ndarray): 잘린 이미지
    """
    height = image.shape[0]
    return image[height - pixels:, :]

def enter_rotary_Is__extract_horizontal_stop_line_region(binary_img, pixel_threshold=75, min_region_height=20):
    """
    가로 방향의 흰색 정지선을 검출하여 해당 영역만 추출합니다.

    :param binary_img: 이진 이미지 (Grayscale, 0과 255로 구성)
    :param pixel_threshold: 한 행에 이 이상 흰 픽셀이 있으면 정지선 후보로 간주
    :param min_region_height: 최소한 이만큼 연속된 행이 있어야 정지선으로 인정
    :return: 검출된 영역 이미지 (없으면 None)
    """
    h, w = binary_img.shape
    horizontal_sums = np.sum(binary_img == 255, axis=1)  # 각 행에서 흰색 픽셀 수 계산

    # 정지선 후보 행의 인덱스들
    candidate_rows = np.where(horizontal_sums > pixel_threshold)[0]

    if len(candidate_rows) == 0:
        return None  # 아무것도 못 찾음

    # 연속된 행 그룹 찾기
    regions = []
    start = candidate_rows[0]
    for i in range(1, len(candidate_rows)):
        if candidate_rows[i] != candidate_rows[i - 1] + 1:
            end = candidate_rows[i - 1]
            if end - start + 1 >= min_region_height:
                regions.append((start, end))
            start = candidate_rows[i]
    # 마지막 구간도 확인
    end = candidate_rows[-1]
    if end - start + 1 >= min_region_height:
        regions.append((start, end))

    if not regions:
        return False
    return True
