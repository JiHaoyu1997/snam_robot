#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum

import search_pattern
from hsv import HSVSpace, from_cv_to_hsv
from map import map
from pid_controller import pid_controller


# Dynamic reconfiguration
from dynamic_reconfigure.server import Server
from vpa_robot_vision.cfg import color_hsvConfig


# Msg
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image 

from vpa_robot_vision.msg import CrossInfo

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
    # Initialization
    def __init__(self) -> None:

        # Initialize the ROS node
        rospy.init_node('visual_detector')

        # Robot name
        self.robot_name = rospy.get_param('~robot_name', 'db19')

        # Simple ACC Function: on or off, default on
        self.acc_mode = bool(rospy.get_param('~acc_on', True))

        # Image Size
        self.image_width = rospy.get_param('~image_width', 320)
        self.image_height = rospy.get_param('~image_height', 240)

        # Default Velocity Setup
        self.v_set = VelocitySet()

        # std_msgs img ==> ros img
        self.bridge = CvBridge()

        # init hsv color spaces for selecting 
        self.color_space_init()

        # 
        self.status_flag_init()

        # Publishers
        self.cv_image_pub = rospy.Publisher("cv_image", Image, queue_size=1)
        self.mask_image_pub = rospy.Publisher("mask_image", Image, queue_size=1)
        self.cmd_vel_from_img_pub = rospy.Publisher('cmd_vel_from_img', Twist, queue_size=1)

        # Subscribers
        self.curr_route_sub = rospy.Subscriber('curr_route', Int8MultiArray, self.curr_route_sub_cb)
        self.image_raw_sub = rospy.Subscriber("robot_cam/image_raw", Image, self.image_raw_sub_cb)

        # Servers
        self.srv_color = Server(color_hsvConfig, self.dynamic_reconfigure_callback_hsv)

        # Clients
        self.inter_boundary_line_detect_pub = rospy.Publisher('inter_boundary_line_detect', CrossInfo, queue_size=1) 

        rospy.loginfo('Visual Detector is Online')

    # Methods
    def status_flag_init(self):
        # Current route (three intersections: last, current and next)
        self.curr_route = [0, 0, 0]

        # Current zone (default buffer area)
        self.current_zone = Zone.BUFFER_AREA

        # self.lane_direction = LaneDirection.RIGHT_HAND 

        # Intersection Boundary Line count
        self.cross_inter_boundary_line_count = 0

        # Boundary Crossing flag
        self.cross = False

        # Enter Conflict Zone flag
        self.enter_conflict_zone = False

        # Stop Operation
        self.stop = False

        # Time lock (prohibit unreasonable status change)
        self.enter_inter_time = 0
        self.left_inter_time  = 0

        # Task
        self.find_task_line = False     # did the robot find the task_line already
        self.task_counter   = 0         # this is to count how many tasks has this specific robot conducted
        self.task_list      = []        # a list of intersections to travel through
        self.ask_task_by_period = False # if the robot will check if there is task at certain period

        self.node_pointer   = 2
        
        # Action
        self.next_action = None        # the next action to perfrom 
        self.inquiry_inter_by_period = False # if teh robot will check if can pass the current intersection
        self.lock_inter_source = True

        self.action_dic = {
            0:'go thur',
            1:'left turn',
            2:'right turn',
            3:'stop'
        }

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

        # Buffer Line HSV - Pink
        self.buffer_line_hsv = HSVSpace(160, 125, 140, 10, 240, 200)

        # Ready Line HSV - Yellow
        self.ready_line_hsv = HSVSpace(100, 80, 255, 80, 255, 150)

        # Intersection Boundary Line HSV - Green
        self.inter_boundary_line_hsv = HSVSpace( 50,  20, 240, 140, 220, 130)

        # guiding lines inside intersections - no dynamic reconfigure
        self._right_guide_hsv = HSVSpace(140, 100, 120,  80, 250, 200)
        self._left_guide_hsv  = HSVSpace(160, 140, 180,  80, 230, 160)
        self._thur_guide_hsv  = HSVSpace( 30,   0, 250, 170, 230, 130)  
        self.inter_guide_line = [self._thur_guide_hsv, self._left_guide_hsv, self._right_guide_hsv]

        # 
        self._acc_aux_hsv     = HSVSpace(150, 110, 180, 100, 255, 120)       

    def curr_route_sub_cb(self, route_msg: Int8MultiArray):
        if route_msg:
            self.curr_route = [route for route in route_msg.data]

    def image_raw_sub_cb(self, data: Image):
        if self.curr_route == [0, 0, 0]:
            rospy.loginfo("Not Start Tracking")
            return
        else:             
            target_x = self.image_width / 2 

        """Step1 CONVERT RAW IMAGE TO HSV IMAGE"""
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

        turn_right_line_mask_img = self.center_line_hsv.apply_mask(cv_hsv_img)
        self.pub_mask_image(mask_img=turn_right_line_mask_img)

        return

        """Step2 BOUNDARY LINE DETECTOR"""
        self.detect_inter_boundary_line(cv_hsv_img=cv_hsv_img) 

        """Step3 FROM HSV IMAGE TO TARGET COORDINATE"""
        # --- BUFFER AREA ---
        # INIT TASK ==> FROM BUFFER TO INTERSECTION          
        if self.curr_route == [6, 6, 2]:
            self.current_zone == Zone.BUFFER_AREA
            # 
            buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(cv_hsv_img=cv_hsv_img, buffer_line_hsv=self.buffer_line_hsv)
            if not buffer_line_x == None:
                cv2.circle(cv_img, (buffer_line_x, buffer_line_y), 5, (255, 100, 0), 5)
                target_x = buffer_line_x
            else:
                target_x = self.image_width / 2 

        # NO TASK ==> STOP AT READY_LINE 
        elif self.curr_route == [2, 6, 6]:
            self.current_zone == Zone.BUFFER_AREA
            # 
            dis2ready = search_pattern.search_line(cv_hsv_img, self.ready_line_hsv)
            if dis2ready > 25:
                self.stop = True
                return

        # --- INTERSECTION AREA ---
        # DEFAULT FIRST ROUTE [6, 6, 2]
        elif self.curr_route[0] == 6 and self.curr_route[1] == 2:
            self.current_zone == Zone.INTERSECTION
            # 
            dis2exit = search_pattern.search_line(cv_hsv_img, self.side_line_hsv)
            self.enter_conflict_zone = dis2exit > 25

            if not self.enter_conflict_zone:                          
                #
                buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(cv_hsv_img=cv_hsv_img, buffer_line_hsv=self.buffer_line_hsv)
                if not buffer_line_x == None:
                    cv2.circle(cv_img, (buffer_line_x, buffer_line_y), 5, (255, 100, 0), 5)
                    target_x = buffer_line_x
                else:
                    target_x = self.image_width / 2 

            else: 
                # 
                self.next_action = map.local_mapper(last=self.curr_route[0], current=self.curr_route[1], next=self.curr_route[2])
                rospy.loginfo(f"Next Action is {self.next_action}")
                line_x = search_pattern.search_inter_guide_line2(self.inter_guide_line[self.next_action], cv_hsv_img, self.next_action)
                if line_x == None:
                    target_x = line_x
                else:
                    target_x = self.image_width / 2
        else:
            self.current_zone == Zone.INTERSECTION
            rospy.loginfo(f"current route is {self.curr_route}")

        self.pub_img(cv_img=cv_img)  
        
        """Step4 FROM TARGET COORDINATE TO TWIST"""
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

        """Step5 PUB TWIST TO DECISION MAKER"""
        # since the cmd_vel is sending on a higher frequency than ACC msg, we estimate the distance to further avoid collsions
        if self.acc_mode:
            v_factor = 1
        else:
            v_factor = 1
        self.pub_cmd_vel_from_img(v_x, omega_z, v_factor)

    def detect_inter_boundary_line(self, cv_hsv_img: Image):
        dis2bound = search_pattern.search_line(cv_hsv_img, self.inter_boundary_line_hsv)
        corss_inter_boundary = dis2bound > 25
        if corss_inter_boundary:
            self.cross_inter_boundary_line_count += 1
            rospy.loginfo(self.cross_inter_boundary_line_count)
            if self.cross_inter_boundary_line_count >= 2 and not self.cross:
                rospy.loginfo(f'{self.robot_name} cross the boundary line between inter{self.curr_route[1]} and inter{self.curr_route[2]}')
                self.cross = True
        else:
            self.cross_inter_boundary_line_count = 0
            self.cross = False

        # 
        cross_msg = CrossInfo()
        cross_msg.cross = self.cross
        cross_msg.robot_name = self.robot_name
        cross_msg.last_inter_id = self.curr_route[1]
        cross_msg.local_inter_id = self.curr_route[2]
        self.inter_boundary_line_detect_pub.publish(cross_msg)
    
    def pub_cmd_vel_from_img(self, v_x, omega_z, v_factor):
        cmd_msg = Twist()
        cmd_msg.linear.x = v_x * v_factor
        cmd_msg.angular.z = omega_z * v_factor
        self.cmd_vel_from_img_pub.publish(cmd_msg)    

    def pub_img(self, cv_img):
        cv_img_copy = cv_img
        cv_img_copy_msg = self.bridge.cv2_to_imgmsg(cv_img_copy, encoding="bgr8")
        cv_img_copy_msg.header.stamp = rospy.Time.now()
        self.cv_image_pub.publish(cv_img_copy_msg)       

    def pub_mask_image(self, mask_img):
        mask_img_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="passthrough")
        mask_img_msg.header.stamp = rospy.Time.now()
        self.mask_image_pub.publish(mask_img_msg)

    def dynamic_reconfigure_callback_hsv(self, config, level):
        
        self.center_line_hsv._h_lower = config.h_lower_1
        self.center_line_hsv._s_lower = config.s_lower_1
        self.center_line_hsv._v_lower = config.v_lower_1
        
        self.center_line_hsv._h_upper = config.h_upper_1
        self.center_line_hsv._s_upper = config.s_upper_1
        self.center_line_hsv._v_upper = config.v_upper_1

        self.side_line_hsv._h_lower = config.h_lower_2
        self.side_line_hsv._s_lower = config.s_lower_2
        self.side_line_hsv._v_lower = config.v_lower_2

        self.side_line_hsv._h_upper = config.h_upper_2
        self.side_line_hsv._s_upper = config.s_upper_2
        self.side_line_hsv._v_upper = config.v_upper_2
        
        self.stop_line_hsv._h_lower = config.h_lower_s
        self.stop_line_hsv._s_lower = config.s_lower_s
        self.stop_line_hsv._v_lower = config.v_lower_s
        self.stop_line_hsv._h_upper = config.h_upper_s
        self.stop_line_hsv._s_upper = config.s_upper_s
        self.stop_line_hsv._v_upper = config.v_upper_s  
        
        return config

if __name__ == '__main__':
    N = RobotVision()
    rospy.spin()  