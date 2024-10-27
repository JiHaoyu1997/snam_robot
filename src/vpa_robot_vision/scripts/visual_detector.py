#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum

import search_pattern
from hsv import HSVSpace, from_cv_to_hsv
from pid_controller import pid_controller

# ROS message types
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image 

from vpa_robot_vision.msg import CrossInfo

# Zone type definition: Buffer Area and Intersection
class Zone(Enum):
    BUFFER_AREA     = 0  # Buffer area: waiting area outside the intersection
    INTERSECTION    = 1  # Intersection

# Velocity setting class
class VelocitySet():    
    def __init__(self) -> None:
        # Define speeds for different zones and lanes
        self.v_f_lane   = 0.3  # Forward lane speed
        self.v_s_lane   = 0.3  # Stop lane speed
        self.v_f_inter  = 0.3  # Intersection forward speed
        self.v_s_inter  = 0.3  # Intersection stop speed
        self.v_f_buffer = 0.3  # Buffer forward speed
        self.v_s_buffer = 0.3  # Buffer stop speed

# Vision detection class
class RobotVision:
    # Initialization
    def __init__(self) -> None:

        # Initialize the ROS node
        rospy.init_node('visual_detector')

        # Robot name
        self.robot_name = rospy.get_param('~robot_name', 'db19')

        # Current route (two intersections: current and next)
        self.curr_route = [0, 0]

        # Current zone (default buffer area)
        self.current_zone = Zone.BUFFER_AREA

        # Intersection boundary line count
        self.cross_inter_boundary_line_count = 0

        # Boundary crossing flag
        self.cross = False

        # Velocity settings
        self.v_set = VelocitySet()

        # Stop flag
        self.stop = False

        # Adaptive Cruise Control (ACC) mode, default is on
        self.acc_mode = bool(rospy.get_param('~acc_on', True))

        # Image bridge (for format conversion)
        self.bridge = CvBridge()

        # Image dimensions
        self.image_width = rospy.get_param('~image_width', 320)
        self.image_height = rospy.get_param('~image_height', 240)

        # Initialize HSV color space parameters
        self.color_space_init()
    
        # Define publishers
        self.cv_image_pub = rospy.Publisher("cv_image", Image, queue_size=1)
        self.mask_image_pub = rospy.Publisher("mask_image", Image, queue_size=1)
        self.cmd_vel_from_img_pub = rospy.Publisher('cmd_vel_from_img', Twist, queue_size=1)

        # Define subscribers
        self.curr_route_sub = rospy.Subscriber('curr_route', Int8MultiArray, self.curr_route_sub_cb)
        self.image_raw_sub = rospy.Subscriber("robot_cam/image_raw", Image, self.image_raw_sub_cb)

        # Publish boundary detection message
        self.inter_boundary_line_detect_pub = rospy.Publisher('inter_boundary_line_detect', CrossInfo, queue_size=1) 

        rospy.loginfo('Visual Detector is Online')

    # Initialize color space
    def color_space_init(self) -> None:
        # Initialize HSV ranges for different colors such as yellow, white, and red for lane lines, stop lines, etc.
        self.center_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_1', 100)),
            h_l=int(rospy.get_param('~h_lower_1', 80)),
            s_u=int(rospy.get_param('~s_upper_1', 255)),
            s_l=int(rospy.get_param('~s_lower_1', 80)),
            v_u=int(rospy.get_param('~v_upper_1', 255)),
            v_l=int(rospy.get_param('~v_lower_1', 150))
        ) 

        # White side line
        self.side_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_2', 100)),
            h_l=int(rospy.get_param('~h_lower_2', 25)),
            s_u=int(rospy.get_param('~s_upper_2', 60)),
            s_l=int(rospy.get_param('~s_lower_2', 0)),
            v_u=int(rospy.get_param('~v_upper_2', 255)),
            v_l=int(rospy.get_param('~v_lower_2', 200))
        ) 

        # Red stop line
        self.stop_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_s', 145)),
            h_l=int(rospy.get_param('~h_lower_s', 110)),
            s_u=int(rospy.get_param('~s_upper_s', 180)),
            s_l=int(rospy.get_param('~s_lower_s', 120)),
            v_u=int(rospy.get_param('~v_upper_s', 235)),
            v_l=int(rospy.get_param('~v_lower_s', 170))
        )

        # HSV parameters for other related color spaces (e.g., buffer lines, ready lines, intersection boundary lines, etc.)

    # Current route subscriber callback
    def curr_route_sub_cb(self, route_msg: Int8MultiArray):
        if route_msg:
            self.curr_route = route_msg.data

    # Image processing callback
    def image_raw_sub_cb(self, data: Image):
<<<<<<< HEAD
        """Step 1: Convert raw image to HSV image"""
=======
        if self.curr_route == [0, 0]:
            return
        
        target_x = self.image_width / 2 

        """Step1 CONVERT RAW IMAGE TO HSV IMAGE"""
        # the function is supposed to be called at about 10Hz based on fps settins  
>>>>>>> 4033c551107489511c21fc7eedd49e0828f08013
        try:
            cv_img_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)        
        
        # Use different parts of the raw image for different tasks
        acc_img = cv_img_raw[0 : int(cv_img_raw.shape[0]/3), :]
        cv_img = cv_img_raw[int(cv_img_raw.shape[0]/4) : cv_img_raw.shape[0], :]

        # Convert to HSV image
        acc_hsv_img = from_cv_to_hsv(acc_img)
        cv_hsv_img = from_cv_to_hsv(cv_img)

        """Step 2: Boundary line detection"""
        self.detect_inter_boundary_line(cv_hsv_img=cv_hsv_img)        

        """Step 3: Extract target coordinates from HSV image"""
        # Buffer area
        if self.curr_route == [6, 6]:
            dis2ready = search_pattern.search_line(cv_hsv_img, self.ready_line_hsv)
            if dis2ready > 25:
                self.stop = True
                return
            
<<<<<<< HEAD
        if self.curr_route[0] == [6]:
=======
        if self.curr_route[0] == 6:
            # 
>>>>>>> 4033c551107489511c21fc7eedd49e0828f08013
            self.current_zone == Zone.BUFFER_AREA
            buffer_line_mask_img = self.buffer_line_hsv.apply_mask(cv_hsv_img)
<<<<<<< HEAD
=======
            self.pub_mask_image(mask_img=buffer_line_mask_img)
            
            # buffer line center ==> target coordinate
>>>>>>> 4033c551107489511c21fc7eedd49e0828f08013
            buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(buffer_line_mask_img)
            
            if buffer_line_x is not None:
                cv2.circle(cv_img, (buffer_line_x, buffer_line_y), 5, (255, 100, 0), 5)
                self.pub_img(cv_img=cv_img)
                target_x = buffer_line_x
            else:
<<<<<<< HEAD
                target_x = self.image_width / 2
=======
                target_x = self.image_width / 2      
>>>>>>> 4033c551107489511c21fc7eedd49e0828f08013

        # Intersection area
        else: 
            self.current_zone == Zone.INTERSECTION
            # Lane detection logic (not implemented yet)
        
        """Step 4: From target coordinates to Twist commands"""
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
            
        """Step 5: Publish Twist message to decision module"""
        v_factor = 1 if self.acc_mode else 1
        self.pub_cmd_vel_from_img(v_x, omega_z, v_factor)

    # Boundary line detection function
    def detect_inter_boundary_line(self, cv_hsv_img: Image):
        dis2bound = search_pattern.search_line(cv_hsv_img, self.inter_boundary_line_hsv)
        corss_inter_boundary = dis2bound > 25

        if corss_inter_boundary:
            self.cross_inter_boundary_line_count += 1

            if self.cross_inter_boundary_line_count >= 3 and not self.cross:
                rospy.loginfo(f'{self.robot_name} cross the boundary line between inter{self.curr_route[0]} and inter{self.curr_route[1]}')
                self.cross = True
    
        else:
            self.cross_inter_boundary_line_count = 0
            self.cross = False

    # Publish Twist message function
    def pub_cmd_vel_from_img(self, v_x, omega_z, factor):
        twist = Twist()
        twist.linear.x = v_x * factor
        twist.angular.z = omega_z * factor
        self.cmd_vel_from_img_pub.publish(twist)

    # Publish boundary detection message function
    def pub_inter_boundary_line_detect(self):
        cross_info = CrossInfo()
        cross_info.inter_id = self.curr_route[0]
        cross_info.robot_id_list = [0, 0] # Robot ID list
        self.inter_boundary_line_detect_pub.publish(cross_info)

    # Image display function
    def display_images(self, img, title='Image'):
        cv2.imshow(title, img)
        cv2.waitKey(1)
    
    # Stop the node
    def stop_node(self):
        rospy.loginfo('Visual Detector Node Terminated.')
        rospy.signal_shutdown('Node Shutdown')    

if __name__ == "__main__":
    visual_detector = RobotVision()
    rospy.spin()
