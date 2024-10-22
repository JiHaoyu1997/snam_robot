#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum

import search_pattern
from hsv import HSVSpace, from_cv_to_hsv
from pid_controller import pid_controller

# Msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image 

class Zone(Enum):
    BUFFER_AREA     = 0 # this is queuing area outside the intersections 
    INTERSECTION    = 1

class VelocitySet():    
    def __init__(self) -> None:
        self.v_f_lane   = 0.3
        self.v_s_lane   = 0.3
        self.v_f_inter  = 0.3
        self.v_s_inter  = 0.3
        self.v_f_buffer = 0.3
        self.v_s_buffer = 0.3 

class RobotVision:
    # Properties
    def __init__(self) -> None:

        # Init ROS Node
        rospy.init_node('visual_detector')

        # Robot Name   
        self.robot_name = rospy.get_param('~robot_name', 'db19')

        # Current Zone
        self.current_zone = Zone.BUFFER_AREA

        # Default Velocity Setup
        self.v_set = VelocitySet()

        # Stop Operation
        self.stop = False

        # No Task List Status
        self.no_task_list = True

        # Simple ACC Function: on or off, default on
        self._acc_mode = bool(rospy.get_param('~acc_on', True))

        # std_msgs img ==> ros img
        self.bridge = CvBridge()

        # Image Size
        self.image_width = rospy.get_param('~image_width', 320)
        self.image_height = rospy.get_param('~image_height', 240)
    
        # Publishers
        self.mask_image_pub = rospy.Publisher("mask_image", Image, queue_size=1)
        self.cv_image_pub = rospy.Publisher("cv_image", Image, queue_size=1)
        self.inter_boundary_line_detect_pub = rospy.Publisher('inter_boundary_line_detect', Bool, queue_size=1)
        self.cmd_vel_from_img_pub = rospy.Publisher('cmd_vel_from_img', Twist, queue_size=1)

        # Subscribers
        self.image_raw_sub = rospy.Subscriber("robot_cam/image_raw", Image, self.image_raw_cb)

        # Servers

        # Clients
        

        rospy.loginfo('Visual Detector is Online')

        # init hsv color spaces for selecting 
        self.color_space_init()

    # Methods
    def color_space_init(self) -> None:
        # HSV space for yellow (center lane line)
        self.center_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_1', 100)),
            h_l=int(rospy.get_param('~h_lower_1', 80)),
            s_u=int(rospy.get_param('~s_upper_1', 255)),
            s_l=int(rospy.get_param('~s_lower_1', 80)),
            v_u=int(rospy.get_param('~v_upper_1', 255)),
            v_l=int(rospy.get_param('~v_lower_1', 150))
        ) 

        # HSV space for white (side lane line)
        self.side_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_2', 100)),
            h_l=int(rospy.get_param('~h_lower_2', 25)),
            s_u=int(rospy.get_param('~s_upper_2', 60)),
            s_l=int(rospy.get_param('~s_lower_2', 0)),
            v_u=int(rospy.get_param('~v_upper_2', 255)),
            v_l=int(rospy.get_param('~v_lower_2', 200))
        ) 

        # HSV space for red (stop line)
        self.stop_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_s', 145)),
            h_l=int(rospy.get_param('~h_lower_s', 110)),
            s_u=int(rospy.get_param('~s_upper_s', 180)),
            s_l=int(rospy.get_param('~s_lower_s', 120)),
            v_u=int(rospy.get_param('~v_upper_s', 235)),
            v_l=int(rospy.get_param('~v_lower_s', 170))
        )

        # Buffer Line HSV
        self.buffer_line_hsv = HSVSpace(160, 125, 140, 10, 240, 200)

        # Ready Line HSV
        self.ready_line_hsv = HSVSpace(100, 80, 255, 80, 255, 150)

        # Intersection Boundary Line HSV
        self.inter_boundary_line_hsv = HSVSpace( 50,  20, 240, 140, 220, 130)

        # # guiding lines inside intersections - no dynamic reconfigure
        # self._right_guide_hsv = HSVSpace(140, 100, 120,  80, 250, 200)
        # self._left_guide_hsv  = HSVSpace(160, 140, 180,  80, 230, 160)
        # self._thur_guide_hsv  = HSVSpace( 30,   0, 250, 170, 230, 130)  

        # self.inter_guide_line = [self._thur_guide_hsv, self._left_guide_hsv, self._right_guide_hsv]

        # self._acc_aux_hsv     = HSVSpace(150, 110, 180, 100, 255, 120)
       
        # self._exit_line_hsv   = HSVSpace( 50,  20, 240, 140, 220, 130)
        
        # self.task_line_hsv    = HSVSpace(100,  50, 255,   3, 255, 150)

    def image_raw_cb(self, data: Image):
        """Step1 HSV IMAGE"""
        # the function is supposed to be called at about 10Hz based on fps settins  
        try:
            cv_img_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)        
        
        # the top 1/3 part of image_raw for acc function
        acc_img = cv_img_raw[0 : int(cv_img_raw.shape[0]/3), :]

        # the bottom 3/4 part of image_raw for tracking function
        cv_img = cv_img_raw[int(cv_img_raw.shape[0]/4) : cv_img_raw.shape[0], :]

        # convert BGR image to HSV image
        acc_hsv_img = from_cv_to_hsv(acc_img)
        cv_hsv_img = from_cv_to_hsv(cv_img)

        """Step2 FROM HSV IMAGE TO TARGET COORDINATE"""
        # BUFFER AREA
        if self.current_zone == Zone.BUFFER_AREA:
            # Buffer Mask Image
            buffer_line_mask_img = self.buffer_line_hsv.apply_mask(cv_hsv_img)
            
            # buffer line center ==> target coordinate
            buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(buffer_line_mask_img)
            
            if not buffer_line_x == None:
                cv2.circle(cv_img, (buffer_line_x, buffer_line_y), 5, (255, 100, 0), 5)
                target_x = buffer_line_x
            else:
                target_x = self.image_width / 2 
            
            # if get new Task List, just along the buffer line to inter.
            if not self.no_task_list:
                # detect intersection boundary line
                self.detect_inter_boundary_line(cv_hsv_img)                    
            # else stop at the ready line
            else:
                # check ready line
                dis2ready = search_pattern.search_line(cv_hsv_img, self.ready_line_hsv)
                if dis2ready > 25:
                    self.stop = True

        # INTERSECTION AREA
        elif self.current_zone == Zone.INTERSECTION:
            # detect lane line
            pass
        
        """Step3 FROM TARGET COORDINATE TO TWIST"""
        if self.stop:
            v_x = 0
            omega_z = 0

        elif self.current_zone == Zone.BUFFER_AREA:
            v_x, omega_z = pid_controller.bufffer_pi_control(
                int(cv_hsv_img.shape[1]/2), 
                target_x, 
                self.v_set.v_f_buffer, 
                self.v_set.v_s_buffer)
            
        elif self.current_zone == Zone.INTERSECTION:
            v_x, omega_z = pid_controller.inter_pi_control(
                int(cv_hsv_img.shape[1]/2), 
                target_x, 
                self.v_set.v_f_inter, 
                self.v_set.v_s_inter)
            
        """Step4 PUB TWIST TO DECISION MAKER"""
        # since the cmd_vel is sending on a higher frequency than ACC msg, we estimate the distance to further avoid collsions
        if self._acc_mode:
            pass
        else:
            v_factor = 1        
        self.pub_cmd_vel_from_img(v_x, omega_z, v_factor) 

    def detect_inter_boundary_line(self, cv_hsv_img: Image):
        dis2bound = search_pattern.search_line(cv_hsv_img, self.inter_boundary_line_hsv)
        corss_inter_boundary = dis2bound > 25
        if corss_inter_boundary:
            self.current_zone = Zone.INTERSECTION
        self.inter_boundary_line_detect_pub.publish(corss_inter_boundary)
    
    def pub_cmd_vel_from_img(self, v_x, omega_z, v_factor):
        cmd = Twist()
        cmd.linear.x = v_x * v_factor
        cmd.angular.z = omega_z * v_factor
        self.cmd_vel_from_img_pub.publish(cmd)
    
    def pub_img(self, cv_img):
        cv_img_copy = cv_img
        cv_img_copy_msg = self.bridge.cv2_to_imgmsg(cv_img_copy, encoding="bgr8")
        cv_img_copy_msg.header.stamp = rospy.Time.now()
        self.cv_image_pub.publish(cv_img_copy_msg)        

    def pub_mask_image(self, mask_img):
        mask_img_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="passthrough")
        mask_img_msg.header.stamp = rospy.Time.now()
        self.mask_image_pub.publish(mask_img_msg)
    

        


if __name__ == '__main__':
    N = RobotVision()
    rospy.spin()      