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


def img_curve(angle_deg,binary_img,a,b,c,y, color=(0, 0, 255), thickness=2):
        # 컬러 이미지로 변환
    if len(binary_img.shape) == 2:
        vis_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
    else:
        vis_img = binary_img.copy()

    # 곡선 좌표 계산
    y_sorted = np.linspace(min(y), max(y), 300).astype(int)
    x_fit = (a * y_sorted ** 2 + b * y_sorted + c).astype(int)

    # 유효 좌표만 남기기
    valid = (x_fit >= 0) & (x_fit < vis_img.shape[1]) & (y_sorted >= 0) & (y_sorted < vis_img.shape[0])
    points = np.array([[x, y] for x, y in zip(x_fit[valid], y_sorted[valid])], dtype=np.int32)

    # 곡선 그리기
    for i in range(1, len(points)):
        cv2.line(vis_img, tuple(points[i - 1]), tuple(points[i]), color, thickness)

    # 각도 표시
    cv2.putText(vis_img, f"Angle: {angle_deg:.2f} deg", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    cv2.imshow("test", vis_img)
    cv2.waitKey(0)

def extract_horizontal_contours(binary_img, aspect_ratio_threshold=1.5, min_contour_len=50):
    """
    이진 이미지에서 수평 방향으로 누운 윤곽선만 추출합니다.

    Parameters:
        binary_img (numpy.ndarray): GRAYSCALE의 이진 이미지
        aspect_ratio_threshold (float): width / height 비율이 이 값보다 크면 수평으로 간주
        min_contour_len (int): 너무 짧은 윤곽선은 제외

    Returns:
        horizontal_contours (list): 수평 윤곽선만 필터링된 결과
    """
    contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    horizontal_contours = []

    for cnt in contours:
        if len(cnt) < min_contour_len:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if w > h and (w / h) > aspect_ratio_threshold:
            horizontal_contours.append(cnt)

    if not horizontal_contours:
        print("수평 윤곽선이 없습니다.")
    else:
        areas = [cv2.contourArea(cnt) for cnt in horizontal_contours]
        max_idx = areas.index(max(areas))
        largest_contour = horizontal_contours[max_idx]

        pts = largest_contour.reshape(-1, 2)
        x = pts[:, 0]
        y = pts[:, 1]

        fit = np.polyfit(y, x, 2)
        a, b, c = fit
        print(f"2차 다항식 계수: a={a:.6f}, b={b:.6f}, c={c:.6f}")

        y_mid = (min(y) + max(y)) // 2
        slope = 2 * a * y_mid + b
        angle_deg = np.degrees(np.arctan(slope))
        
        print(f"중간 y에서 접선 기울기의 각도: {angle_deg:.2f}도")
        img_curve(angle_deg,binary_img,a,b,c,y)
    return horizontal_contours

def split_image_vertically(image):
    """
    이미지를 수직(상하)으로 절반으로 나누어 반환합니다.

    Parameters:
        image (numpy.ndarray): 입력 이미지

    Returns:
        top_half (numpy.ndarray): 상단 절반
        bottom_half (numpy.ndarray): 하단 절반
    """
    height = image.shape[0]
    top_half = image[:height // 2, :]
    bottom_half = image[height // 2:, :]
    return top_half, bottom_half

def extract_horizontal_stop_line_region(binary_img, pixel_threshold=75, min_region_height=20):
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
        return None

    # 가장 긴 구간 선택
    best_region = max(regions, key=lambda r: r[1] - r[0])
    top, bottom = best_region
    # 잘라서 반환
    return binary_img[top:bottom + 1, :]

def crop_bottom(image, pixels=100):
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

# 이미지 경로
img_path = './test_img/rotary/stop_bin_img.png'
binary = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
_start = time.time()
cropped = crop_bottom(binary, 100)
elapsed = time.time() - _start
print(f"[Timer] 경과 시간: {elapsed:.6f}초")
cv2.imshow("Cropped Image", cropped)
cv2.waitKey(0)
cv2.destroyAllWindows()
# # 절반으로 나누기


# 사용할지 모르니 일단 넣기
        
    # def detect_nothing(self):
    #     center = round(self.img_x * 0.140625)
    #     self.nothing_left_x_base = center
    #     self.nothing_right_x_base = self.img_x - center
    #     self.nothing_pixel_left_x = np.zeros(self.nwindows) + self.nothing_left_x_base 
    #     self.nothing_pixel_right_x = (np.zeros(self.nwindows) + self.nothing_right_x_base)
    #     self.nothing_pixel_y =  [round(self.window_height / 2) * index for index in range(0,self.nwindows)]
    
    # def window_search(self,binary_line):
    #     # 2. 에지 검출 (Canny)
    #     edges = cv2.Canny(binary_line, 0, 50)

    #     # 3. 윤곽선 추출
    #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #     result = cv2.cvtColor(binary_line, cv2.COLOR_GRAY2BGR)
    #     h, w = binary_line.shape

    #     # 3. 원본 또는 컬러 버전으로 컨투어 그리기 위해 복사
    #     contour_img = cv2.cvtColor(binary_line, cv2.COLOR_GRAY2BGR)

    #     # 4. 윤곽선 시각화
    #     cv2.drawContours(contour_img, contours, -1, (0, 0, 255), 2)  # 빨간색, 두께 2
    #     cv2.imshow("Contours", contour_img)
    #     for cnt in contours:
    #         if len(cnt) < 200:
    #             continue  # 너무 짧은 건 제외

    #         # 4. contour 점을 x, y로 분리
    #         points = cnt.squeeze()
    #         x = points[:, 0]
    #         y = points[:, 1]

    #         # 5. 다항식 피팅 (2차 곡선)
    #         poly = np.polyfit(y, x, 2)  # y 기준으로 x를 추정 → 화면상 수직 방향 기준

    #         # 6. 예측 좌표로 곡선 그리기
    #         y_fit = np.linspace(0, h - 1, h)
    #         x_fit = poly[0] * y_fit ** 2 + poly[1] * y_fit + poly[2]
    #         for x_, y_ in zip(x_fit.astype(int), y_fit.astype(int)):
    #             if 0 <= x_ < w and 0 <= y_ < h:
    #                 result[y_, x_] = (0, 255, 0)

    #         # 7. 휘는 방향 판단 (최고차항 계수)
    #         curvature = poly[0]
    #         direction = "좌회전" if curvature < 0 else "우회전"
    #         print(f"곡선 방향: {direction}, 곡률 계수: {curvature:.4f}")
            
    #     return result
    