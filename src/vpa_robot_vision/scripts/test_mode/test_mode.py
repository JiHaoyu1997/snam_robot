#!/usr/bin/env python3

import os
import sys
import rospy

from hsv.hsv import HSVSpace, HSV_RANGES
from hsv import search_pattern

script_dir  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if script_dir not in sys.path:
    sys.path.append(script_dir)

try:
    from visual_detector import RobotVision
except ImportError as e:
    print(e)

class TestMode:
    def __init__(self, robot_vision: RobotVision) -> None:
        self.robot_vison = robot_vision

    def handle_test_mode(self, cv_img, cv_hsv_img):
        if self.robot_vison.test_mode == 'rightcircle':
            self.go_thur_right_circle(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

    def go_thur_right_circle(self, cv_img, cv_hsv_img):
        action = 2
        in_lane = True

        dis2red = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=HSV_RANGES['red'])
        if dis2red > 30:
            in_lane = False
            rospy.loginfo("Conflict Zone")

        dis2green = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=HSV_RANGES['green'])
        if dis2green > 30:
            in_lane = True
            rospy.loginfo("Lane Zone")      

        if in_lane:
            target_x, cv_img = self.robot_vison.find_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
            v_x, omega_z = self.robot_vison.calculate_velocity(target_x=target_x)

            hsv_image1 = cv_hsv_img
            hsv_image2 = cv_hsv_img
            mask1 = self.robot_vison.center_line_hsv.apply_mask(hsv_image1)
            mask2 = self.robot_vison.side_line_hsv.apply_mask(hsv_image2)
            mask_img = mask1 + mask2

        else:
        
            target_x, cv_img = self.robot_vison.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, hsv_space=HSV_RANGES['orange'], action=action)
            v_x, omega_z = self.robot_vison.calculate_velocity(target_x=target_x)
            mask_img = HSV_RANGES['orange'].apply_mask(cv_hsv_img)

        self.robot_vison.pub_cmd_vel_from_img(v_x, omega_z)             
        self.robot_vison.pub_cv_img(cv_img=cv_img)
        self.robot_vison.pub_mask_img(mask_img=mask_img)     

        return
