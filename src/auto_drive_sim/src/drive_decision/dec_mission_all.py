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
import json
from std_msgs.msg import String
from utils import check_timer
from time import *
import PIDController
import json
MIN_Y = 0
MAX_Y = 1

class DecMissionAll:
    def __init__(self):
        print(f"DecMissionAll start")
        self.pub_init()
        self.car_mission_status = [0,1,2,3,4,5]
        self.current_car_mission = 2
        self.car_lane_init()
        self.mission4_init()

    def pub_init(self):
        self.motor_pub = rospy.Publisher('/commands/motor/ctrl', Float64, queue_size=1)
        self.motor_cmd_msg_pub = Float64()
        self.servo_pub = rospy.Publisher('/commands/servo/ctrl', Float64, queue_size=1)
        self.servo_cmd_msg_pub = Float64()
        self.rate_motor = rospy.Rate(33)
        self.rate_motor = rospy.Rate(33)
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = None,None,None,None,None
        rospy.Subscriber("/perception/camera", String, self.CB_camera_info, queue_size=1)
        
    def CB_camera_info(self,msg):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = None,None,None,None,None
        try:
            data = json.loads(msg.data)
            for item in data:
                print(item)
            self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = data[0],data[1],data[2],data[3],data[4]
        except Exception as e:
            print("복원 실패:", e)
    def car_lane_init(self):
        self.is_target_car_lane_1 = True
        self.is_target_car_lane_1 = False
        self.car_lane_number = {"null" : -1, "forward" : 0, "left" : 1, "right" : 2}
        self.current_car_lane_number = -1
        self.prev_car_lane_number = -1
        self.car_lane_env = {"null" : -1, "forward" : 0, "left" : 1, "right" : 2}
        self.current_car_lane_env = -1
        self.prev_car_lane_env = -1
        
        self.stop_line = None
        self.yellow_left_lane = None
        self.yellow_right_lane = None
        self.white_left_lane = None
        self.white_right_lane = None 
        self.current_left_lane = None
        self.current_right_lane = None
        
        self.curve_degree = 50
        self.car_center_pixel = 320
        self.car_steer_per_pixel = 1 / 640
        self.pid_steer = PIDController.PIDController()
        # self.pid_steer = PIDController(Kp=0.1, Ki=0.001, Kd=0.000001)
        
        self.max_speed = 4
        self.min_speed = 2
        self.max_steer = 19.5
        self.min_steer = -19.5
        self.total_steer = self.max_steer - self.min_steer
        self.pre_spped = 1

    def mission4_init(self):
        self.mi4_stop_flag = False
        self.mi4_in_flag = False
        self.mi4_out_flag = False
    
    def check_lanes_number(self):
        if self.is_target_car_lane_1:
            if self.yellow_left_lane != [] and self.white_right_lane != []:
                self.current_left_lane = self.yellow_left_lane
                self.current_right_lane = self.white_right_lane
                self.current_car_lane_number = self.car_lane_number["forward"]
            elif self.yellow_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.white_left_lane == [] and self.white_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.white_left_lane != [] and self.white_right_lane == []:
                self.current_car_lane_number = self.car_lane_number["left"]
            elif self.white_left_lane != [] and self.white_right_lane != []:
                if self.prev_car_lane_number == self.car_lane_number["right"]:
                    self.current_car_lane_number = self.car_lane_number["right"]
                else:
                    self.current_car_lane_number = self.car_lane_number["left"]
            else:
                self.current_car_lane_number = self.car_lane_number["null"]
        else:
            if self.yellow_left_lane != [] and self.white_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.yellow_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.white_left_lane == [] and self.white_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.white_left_lane != [] and self.white_right_lane == []:
                self.current_car_lane_number = self.car_lane_number["left"]
            elif self.white_left_lane != [] and self.white_right_lane != []:
                if self.white_left_lane != [] and self.white_right_lane != []:
                    self.current_left_lane = self.white_left_lane
                    self.current_right_lane = self.white_right_lane
                    self.current_car_lane_number = self.car_lane_number["forward"]
                else:
                    self.current_car_lane_number = self.car_lane_number["right"]
            else:
                self.current_car_lane_number = self.car_lane_number["null"]
        self.prev_car_lane_number = self.current_car_lane_number
    def check_lanes_env(self):
        self.prev_car_lane_env = self.current_car_lane_env
        if self.current_car_lane_number == self.car_lane_number["forward"]:
            detect_lane_center = self.detect_center()
            point_direction = self.car_center_pixel - detect_lane_center
            if point_direction > self.curve_degree:
                self.current_car_lane_env = self.car_lane_env["left"]
                pass
            elif point_direction < self.curve_degree:
                self.current_car_lane_env = self.car_lane_env["right"]
                pass
            else:
                self.current_car_lane_env = self.car_lane_env["forward"]
                pass
        else:
            self.current_car_lane_env = self.car_lane_env["null"]
    def check_lanes_status(self):
        self.check_lanes_number()
        self.check_lanes_env()
    
    def move_left(self):
        self.motor_pub.publish(8)
        self.servo_pub.publish(-19.5)
    def move_right(self):
        self.motor_pub.publish(8)
        self.servo_pub.publish(19.5)
    def move_null(self):
        self.motor_pub.publish(8)
        self.servo_pub.publish(-0)
        
    def detect_center(self):
        # lane_centers = []
        # for i in range(3):
        #     left = self.current_left_lane[i] if len(self.current_left_lane) > i else None
        #     right = self.current_right_lane[i] if len(self.current_right_lane) > i else None

        #     if left is not None and right is not None:
        #         center = (left + right) / 2.0
        #     elif left is not None:
        #         center = left
        #     elif right is not None:
        #         center = right
        #     else:
        #         continue  # 둘 다 없으면 스킵

        #     lane_centers.append(center)

        # if lane_centers:
        #     detect_lane_center = sum(lane_centers) / len(lane_centers)
        # else:
        #     detect_lane_center = (self.current_left_lane[0] + self.current_right_lane[0]) / 2.0
        
        alpha = 0.85
        start_center = (self.current_left_lane[0] + self.current_right_lane[0]) / 2.0
        end_center = (self.current_left_lane[-1] + self.current_right_lane[-1]) / 2.0
        detect_lane_center = start_center * alpha + (1 - alpha) * end_center
        
        detect_lane_center = (self.current_left_lane[0] + self.current_right_lane[0]) / 2.0
        return detect_lane_center
    def moveByLine(self):
        if self.current_car_lane_number == self.car_lane_number["null"]:
            self.move_null()
        elif self.current_car_lane_number == self.car_lane_number["left"]:
            self.move_left()
        elif self.current_car_lane_number == self.car_lane_number["right"]:
            self.move_right()
        else:
            # 1. 차선 중심 계산
            detect_lane_center = self.detect_center()
            # detect_lane_center = (self.current_left_lane[-1] + self.current_right_lane[-1]) / 2.0
            # 2. 차선 중심과 차량 중심의 픽셀 오차
            pixel_error = detect_lane_center - self.car_center_pixel  # +면 우측, -면 좌측
            # 3. 조향각 계산 (픽셀 오차 → 각도로 환산)
            if self.pre_spped > 7:
                pixel_error = pixel_error * self.car_steer_per_pixel * self.max_steer
            else:
                print(f"steersteersteersteersteersteersteersteer {pixel_error}")
                pixel_error = pixel_error * self.car_steer_per_pixel * self.total_steer * 3
                print(f"steersteersteersteersteersteersteersteer {pixel_error}")
            steer = self.pid_steer.compute(pixel_error)
            # 클리핑 (조향각 범위 제한)
            steer = max(min(steer, self.max_steer), self.min_steer)
            # 4. 속도 계산 (회전이 클수록 느려짐)
            steer_ratio = abs(steer) / self.max_steer  # 0 ~ 1
            speed = self.max_speed * (1 - steer_ratio)  # 회전 클수록 속도 감소
            speed = max(speed, self.min_speed)
            self.pre_spped = speed
            # 5. 결과 저장 혹은 publish
            self.motor_pub.publish(speed)
            self.servo_pub.publish(steer)
            print(f"[INFO] steer: {steer:.2f} deg, speed: {speed:.2f} km/h")
        
    def check_mission_change(self):
        if self.prev_car_lane_env == self.car_lane_env["left"] and self.current_car_lane_env == self.car_lane_env["forward"]:
            self.current_car_mission = 2

    def mission2_ctrl(self):
        self.is_target_car_lane_1 = False
        # self.is_target_car_lane_1 = True
        # self.check_mission_change()
        # ------------------------- 
        # 각자 해야하는 제어가 있을 것이다.
        # 미션 2 레이더 값이 있다. 전방에 뭐 있으면 걸고 거기서 제어를 한다.
        
        self.moveByLine() # 차선 따라가는 함수

    def mission3_ctrl(self):
        self.motor_pub.publish(0)

    def stop_time(self):
        self.motor_pub.publish(0)
        self.servo_pub.publish(0)
        sleep(2)

    def fast_hardcoding_mission4(self):
        if self.mi4_out_flag:
            self.moveByLine() # 차선 따라가는 함수
        elif self.mi4_in_flag:
            self.motor_pub.publish(8)
            self.servo_pub.publish(-8.5)
            sleep(0.16)
            self.motor_pub.publish(7)
            self.servo_pub.publish(19.5)
            sleep(0.425)
            self.mi4_out_flag = True
            self.stop_time()
        elif self.mi4_stop_flag:
            self.motor_pub.publish(8)
            self.servo_pub.publish(0)
            sleep(0.275)
            self.motor_pub.publish(8.0)
            self.servo_pub.publish(19.5)
            sleep(0.4)
            self.mi4_in_flag = True
        elif self.stop_line[MAX_Y] > 440:
            self.mi4_stop_flag =True
            self.stop_time()
        else:
            self.moveByLine() # 차선 따라가는 함수
    def mission4_ctrl(self):
        self.is_target_car_lane_1 = False
        self.fast_hardcoding_mission4()

    def processing(self):
        while not rospy.is_shutdown():
            start = time()
            if self.stop_line is None:
                continue
            self.check_lanes_status()
            end = time()
            print(f"hihihi")
            if self.car_mission_status[self.current_car_mission] == 1:
                pass
            elif self.car_mission_status[self.current_car_mission] == 2:
                print(f"mission 2")
                print(f"self.current_car_lane_number {self.current_car_lane_number}")
                print(f"self.current_left_lane, {self.current_left_lane}, self.current_right_lane, {self.current_right_lane}")
                self.mission2_ctrl()
                pass
            elif self.car_mission_status[self.current_car_mission] == 3:
                print(f"mission 3")
                self.mission3_ctrl()
            elif self.car_mission_status[self.current_car_mission] == 4:
                print(f"mission 4")
                self.mission4_ctrl()
            elif self.car_mission_status[self.current_car_mission] == 5:
                pass
            
            # print(f"time2 {end - start}")
            self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = None,None,None,None,None
