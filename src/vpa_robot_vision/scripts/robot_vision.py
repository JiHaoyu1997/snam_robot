#! /usr/bin/env python3

import rospy

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from hsv import HSVSpace, from_cv_to_hsv
from search_pattern import search_line

# Msg
from std_msgs.msg import Bool
from sensor_msgs.msg import Image  

class RobotVision:
    # Properties
    def __init__(self) -> None:

        # Init ROS Node
        rospy.init_node('robot_vision')

        self._bridge = CvBridge()   
        self.robot_name = rospy.get_param('~robot_name', 'db19')

        # HSV
        self.exit_line_hsv = HSVSpace( 50,  20, 240, 140, 220, 130)
        self.enter_line_hsv = HSVSpace( 50,  20, 240, 140, 220, 130)
    
        # Publishers
        self.mask_image_pub = rospy.Publisher("mask_image", Image, queue_size=1)
        self.cv_image_pub = rospy.Publisher("cv_image", Image, queue_size=1)
        self.enter_line_detect_pub = rospy.Publisher('enter_line_detect', Bool, queue_size=1)

        # Subscribers
        self.image_raw_sub = rospy.Subscriber("robot_cam/image_raw", Image, self.image_raw_cb)

        # Servers

        # ServiceProxy

        rospy.loginfo('Robot Vision is Online')

    # Methods    
    def image_raw_cb(self, data: Image):
        # the function is supposed to be called at about 10Hz based on fps settins  
        try:
            cv_image_raw = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)        
        
        # the top 1/3 part of image_raw for acc function
        acc_image = cv_image_raw[0 : int(cv_image_raw.shape[0]/3), :]

        # the bottom 3/4 part of image_raw for tracking function
        cv_image = cv_image_raw[int(cv_image_raw.shape[0]/4) : cv_image_raw.shape[0], :]

        # convert BGR image to HSV image
        acc_hsv_image = from_cv_to_hsv(acc_image)
        cv_hsv_image = from_cv_to_hsv(cv_image)

        # # result image pub
        # print(cv_image.shape)
        # cv_image_copy = cv_image[90:180]
        # cv_image_copy_msg = self._bridge.cv2_to_imgmsg(cv_image_copy, encoding="bgr8")
        # cv_image_copy_msg.header.stamp = rospy.Time.now()
        # self.cv_image_pub.publish(cv_image_copy_msg)

        # enter line mask image pub
        enter_line_mask_image = self.enter_line_hsv.apply_mask(cv_hsv_image)
        enter_line_mask_msg = self._bridge.cv2_to_imgmsg(enter_line_mask_image, encoding="passthrough")
        enter_line_mask_msg.header.stamp = rospy.Time.now()
        self.mask_image_pub.publish(enter_line_mask_msg)

        # enter line detect        
        dis2enter = search_line(cv_hsv_image, self.enter_line_hsv)                   
        corss_enter_line = True if dis2enter > 20 else False
        self.enter_line_detect_pub.publish(corss_enter_line)
        # print(corss_enter_line)

    def enter_lane_detect(self, hsv_image, ):
        pass


if __name__ == '__main__':
    N = RobotVision()
    rospy.spin()      