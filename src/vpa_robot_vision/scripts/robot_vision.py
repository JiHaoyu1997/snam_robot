#! /usr/bin/env python3

import rospy
from enum import Enum

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from hsv import HSVSpace, from_cv_to_hsv
from search_pattern import search_line

# Msg
from std_msgs.msg import Bool
from sensor_msgs.msg import Image 

class Zone(Enum):
    BUFFER_AREA     = 0 # this is queuing area outside the intersections 
    INTERSECTION    = 1

class RobotVision:
    # Properties
    def __init__(self) -> None:

        # Init ROS Node
        rospy.init_node('robot_vision')

        # std_msgs img ==> ros img
        self._bridge = CvBridge()

        # Self Zone
        self.current_zone = Zone.BUFFER_AREA

        # Get Robot Name   
        self.robot_name = rospy.get_param('~robot_name', 'db19')
    
        # Publishers
        self.mask_image_pub = rospy.Publisher("mask_image", Image, queue_size=1)
        self.cv_image_pub = rospy.Publisher("cv_image", Image, queue_size=1)
        self.enter_line_detect_pub = rospy.Publisher('enter_line_detect', Bool, queue_size=1)

        # Subscribers
        self.image_raw_sub = rospy.Subscriber("robot_cam/image_raw", Image, self.image_raw_cb)

        # Servers

        # ServiceProxy

        rospy.loginfo('Robot Vision is Online')

        # init hsv color spaces for selecting 
        self._color_space_init()

    # Methods
    def _color_space_init(self) -> None:

        # Intersection Boundary Line HSV
        self.inter_boundary_line_hsv = HSVSpace( 50,  20, 240, 140, 220, 130)

    def image_raw_cb(self, data: Image):
        # the function is supposed to be called at about 10Hz based on fps settins  
        try:
            cv_img_raw = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)        
        
        # the top 1/3 part of image_raw for acc function
        acc_img = cv_img_raw[0 : int(cv_img_raw.shape[0]/3), :]

        # the bottom 3/4 part of image_raw for tracking function
        cv_img = cv_img_raw[int(cv_img_raw.shape[0]/4) : cv_img_raw.shape[0], :]

        # convert BGR image to HSV image
        acc_hsv_img = from_cv_to_hsv(acc_img)
        cv_hsv_img = from_cv_to_hsv(cv_img)

        # # pub mask image
        #     # pub result image
        # print(cv_img.shape)
        # cv_img_copy = cv_img[90:180]
        # cv_img_copy_msg = self._bridge.cv2_to_imgmsg(cv_img_copy, encoding="bgr8")
        # cv_img_copy_msg.header.stamp = rospy.Time.now()
        # self.cv_image_pub.publish(cv_img_copy_msg)

        #     # pub intersection boundary line mask image
        # inter_boundary_line_mask_img = self.inter_boundary_line_hsv.apply_mask(cv_hsv_img)
        # inter_boundary_line_mask_img_msg = self._bridge.cv2_to_imgmsg(inter_boundary_line_mask_img, encoding="passthrough")
        # inter_boundary_line_mask_img_msg.header.stamp = rospy.Time.now()
        # self.mask_image_pub.publish(inter_boundary_line_mask_img_msg)

        if self.current_zone == Zone.BUFFER_AREA:

            pass

        elif self.current_zone == Zone.INTERSECTION:
            
            pass
            
        # detect intersection boundary line        
        dis2enter = search_line(cv_hsv_img, self.inter_boundary_line_hsv)                   
        corss_enter_line = True if dis2enter > 20 else False
        self.enter_line_detect_pub.publish(corss_enter_line)
        # print(corss_enter_line)

    def detect_inter_boundary_line(self, hsv_image, ):
        pass


if __name__ == '__main__':
    N = RobotVision()
    rospy.spin()      