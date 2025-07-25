#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy


class DecMain:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('DecMainNode', anonymous=True)

if __name__ == '__main__':
    try:
        node = DecMain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass