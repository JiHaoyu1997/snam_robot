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

        #
        self.test_mode = bool(rospy.get_param('~test_mode', True))

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
        #  Target to follow
        self.target_x = self.image_width / 2 

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
            1:'turn left',
            2:'turn right',
            3:'stop'
        }

    def color_space_init(self) -> None:
        # 
        self.center_x = rospy.get_param('~center_x', 160)
        self.center_y = rospy.get_param('~center_y', 90)

        # HSV space for Yellow (center lane line)
        self.center_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_1', 0)),
            h_l=int(rospy.get_param('~h_lower_1', 0)),
            s_u=int(rospy.get_param('~s_upper_1', 0)),
            s_l=int(rospy.get_param('~s_lower_1', 0)),
            v_u=int(rospy.get_param('~v_upper_1', 0)),
            v_l=int(rospy.get_param('~v_lower_1', 0))
        ) 

        # HSV space for White (side lane line)
        self.side_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_2', 0)),
            h_l=int(rospy.get_param('~h_lower_2', 0)),
            s_u=int(rospy.get_param('~s_upper_2', 0)),
            s_l=int(rospy.get_param('~s_lower_2', 0)),
            v_u=int(rospy.get_param('~v_upper_2', 0)),
            v_l=int(rospy.get_param('~v_lower_2', 0))
        ) 

        # HSV space for Red (stop line)
        self.stop_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_s', 0)),
            h_l=int(rospy.get_param('~h_lower_s', 0)),
            s_u=int(rospy.get_param('~s_upper_s', 0)),
            s_l=int(rospy.get_param('~s_lower_s', 0)),
            v_u=int(rospy.get_param('~v_upper_s', 0)),            
            v_l=int(rospy.get_param('~v_lower_s', 0))
        )

        # Buffer Line HSV - Pink
        self.buffer_line_hsv = HSVSpace(h_u=190, h_l=150, s_u=255, s_l=60, v_u=255, v_l=185)
        self.overexposed_buffer_line_hsv = HSVSpace(h_u=20, h_l=0, s_u=55, s_l=45, v_u=255, v_l=205)

        # Ready Line HSV - Yellow
        self.ready_line_hsv = self.side_line_hsv

        # Intersection Boundary Line HSV - Green
        self.inter_boundary_line_hsv = HSVSpace(h_u=95, h_l=65, s_u=255, s_l=150, v_u=200, v_l=70)

        # guiding lines inside intersections - no dynamic reconfigure
        self.right_guide_hsv = HSVSpace(h_u=25, h_l=4, s_u= 255, s_l=100, v_u=255, v_l=150)
        self.left_guide_hsv = HSVSpace(h_u=175, h_l=145, s_u= 200, s_l=50, v_u=255, v_l=100)
        self.thur_guide_hsv = HSVSpace(h_u=125, h_l=95, s_u=255, s_l=180, v_u=255, v_l=120)  
        self.inter_guide_line = [self.thur_guide_hsv, self.left_guide_hsv, self.right_guide_hsv]

        # 
        self._acc_aux_hsv     = HSVSpace(150, 110, 180, 100, 255, 120)       

    def curr_route_sub_cb(self, route_msg: Int8MultiArray):
        if route_msg:
            self.curr_route = [route for route in route_msg.data]
        else:
            self.curr_route = [0 , 0, 0]

    def adjust_gamma(self, cv_img, gamma=1.0):
        invGamma = 1.0 / gamma
        lookup_table = np.array([ (i /255.0) ** invGamma * 255 for i in range(256)]).astype("uint8")
        return cv2.LUT(cv_img, lookup_table)

    def image_raw_sub_cb(self, data: Image):
        if self.curr_route == [0, 0, 0]:
            rospy.loginfo("Not Start Tracking")
            return

        """Step1 CONVERT RAW IMAGE TO HSV IMAGE"""
        # the function is supposed to be called at about 10Hz based on fps settins  
        try:
            cv_img_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)        

        # the top 1/3 part of image_raw for acc function
        acc_img = cv_img_raw[0 : int(cv_img_raw.shape[0]/3), :]

        # the bottom 3/4 part of image_raw for tracking function
        cv_img_raw2 = cv_img_raw[int(cv_img_raw.shape[0]/4) : cv_img_raw.shape[0], :]

        # Image Operation
        cv_img = self.adjust_gamma(cv_img=cv_img_raw2, gamma=0.5)

        # convert BGR image to HSV image
        acc_hsv_img = from_cv_to_hsv(acc_img)
        cv_hsv_img = from_cv_to_hsv(cv_img)

        if self.test_mode:
            self.test_mode_func(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
            return

        """Step2 BOUNDARY LINE DETECTOR"""
        self.detect_inter_boundary_line(cv_hsv_img=cv_hsv_img) 

        """Step3 FROM HSV IMAGE TO TARGET COORDINATE"""
        # --- BUFFER AREA ---
        
        # NO TASK ==> STOP AT READY_LINE 
        if self.curr_route == [2, 6, 6]:
            self.current_zone = Zone.BUFFER_AREA
            dis2ready = search_pattern.search_line(cv_hsv_img, self.ready_line_hsv)
            if dis2ready > 25:
                self.stop = True
                return
            else:
                target_x = self.get_target_from_buffer_line(cv_img=cv_img, cv_hsv_img=cv_hsv_img)         
            
        # INIT TASK ==> FROM BUFFER TO INTERSECTION          
        elif self.curr_route == [6, 6, 2]:
            self.current_zone = Zone.BUFFER_AREA
            target_x = self.get_target_from_buffer_line(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        # --- INTERSECTION AREA ---

        # DEFAULT FIRST ROUTE [6, 2, X]
        elif self.curr_route[0] == 6 and self.curr_route[1] == 2:
            self.current_zone = Zone.INTERSECTION
            dis2inside = search_pattern.search_line(cv_hsv_img, self.side_line_hsv)
            if dis2inside > 25:
                self.enter_conflict_zone = True

            if not self.enter_conflict_zone:                          
                #
                target_x = self.get_target_from_buffer_line(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
            else: 
                # 
                target_x = self.get_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        # GENERAL ROUTE
        else:
            target_x = self.cross_intersection(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        self.target_x = target_x       
        # mask_img = self.left_guide_hsv.apply_mask(cv_hsv_img)
        # self.pub_mask_img(mask_img=mask_img)    
        self.pub_cv_img(cv_img=cv_img)
        
        """Step4 FROM TARGET COORDINATE TO TWIST"""
        if self.stop:
            v_x = 0
            omega_z = 0

        elif self.current_zone == Zone.BUFFER_AREA:
            v_x, omega_z = pid_controller.bufffer_pi_control(
                int(cv_hsv_img.shape[1]/2), 
                self.target_x, 
                self.v_set.v_f_buffer, 
                self.v_set.v_s_buffer)            

        elif self.current_zone == Zone.INTERSECTION:
            v_x, omega_z = pid_controller.inter_pi_control(
                int(cv_hsv_img.shape[1]/2), 
                self.target_x, 
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
        dis2bound = search_pattern.search_line(cv_hsv_img, self.inter_boundary_line_hsv, top_line=0)
        corss_inter_boundary = dis2bound > 25
        if corss_inter_boundary:
            self.cross_inter_boundary_line_count += 1
            if self.cross_inter_boundary_line_count >= 2 and not self.cross:
                rospy.loginfo(f'{self.robot_name} cross the boundary line between inter{self.curr_route[1]} and inter{self.curr_route[2]}')
                self.cross = True
                self.enter_conflict_zone = False
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
    
    def cross_intersection(self, cv_img, cv_hsv_img):
        self.current_zone = Zone.INTERSECTION
        # rospy.loginfo(f"current route is {self.curr_route}")
        # check if conflict zone
        self.detect_conflict_boundary_line(cv_hsv_img=cv_hsv_img)
        # lane
        if not self.enter_conflict_zone:
            target_x = self.get_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
        # conflict zone
        else:
            rospy.loginfo(f"Enter Conflict Zone")
            target_x = self.get_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        return target_x

    def detect_conflict_boundary_line(self, cv_hsv_img):
        dis2conflict = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.stop_line_hsv)
        # print(dis2conflict)
        if dis2conflict > 30:
            self.enter_conflict_zone = True
    
    def get_target_from_buffer_line(self, cv_img, cv_hsv_img):
        buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(cv_hsv_img=cv_hsv_img, buffer_line_hsv=self.buffer_line_hsv)
        if buffer_line_x == None:
            buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(cv_hsv_img=cv_hsv_img, buffer_line_hsv=self.overexposed_buffer_line_hsv)
        
        if not buffer_line_x == None:
            cv2.circle(cv_img, (buffer_line_x, buffer_line_y), 5, (255, 100, 0), 5)
        else:
            buffer_line_x = self.image_width * 2 / 5
        target_x = buffer_line_x
        return target_x

    def get_target_to_cross_lane(self, cv_img, cv_hsv_img):
        target_x = search_pattern.search_lane_center(self.center_line_hsv, self.side_line_hsv, cv_hsv_img, is_yellow_left=True)
        if target_x == None:
            target_x = self.image_width / 2
        cv2.circle(cv_img, (int(target_x), int(cv_hsv_img.shape[0]/2)), 5, (0, 255, 0), 5)
        return target_x
    
    def get_target_to_cross_conflict(self, cv_img, cv_hsv_img):
        self.next_action = map.local_mapper(last=self.curr_route[0], current=self.curr_route[1], next=self.curr_route[2])
        rospy.loginfo(f"Next Action is {self.action_dic[self.next_action]}")
        target_x = search_pattern.search_inter_guide_line2(self.inter_guide_line[self.next_action], cv_hsv_img, self.next_action)
        if target_x == None:
            target_x = self.image_width / 2
        cv2.circle(cv_img, (int(target_x), int(cv_hsv_img.shape[0]/2)), 5, (255, 255, 0), 5)
        return target_x

    def pub_cmd_vel_from_img(self, v_x, omega_z, v_factor):
        cmd_msg = Twist()
        cmd_msg.linear.x = v_x * v_factor
        cmd_msg.angular.z = omega_z * v_factor
        self.cmd_vel_from_img_pub.publish(cmd_msg)
    
    def test_mode_func(self, cv_img, cv_hsv_img):
        # Calculate the center coordinates as integers
        # center_x = int(cv_hsv_img.shape[1] / 2)
        # center_y = int(cv_hsv_img.shape[0] / 2)
        center_x = self.center_x
        center_y = self.center_y

        # Draw a circle at the center
        cv2.circle(cv_img, (center_x, center_y), 5, (0, 0, 255), 1)

        # Draw horizontal line
        cv2.line(cv_img, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 1)

        # Draw vertical line
        cv2.line(cv_img, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 1)

        # Log the HSV value at the center point
        rospy.loginfo("Point HSV Value is %s" % cv_hsv_img[center_y, center_x])
        self.pub_cv_img(cv_img=cv_img)

        # Apply the mask and publish the masked image
        turn_right_line_mask_img = self.center_line_hsv.apply_mask(cv_hsv_img)
        self.pub_mask_img(mask_img=turn_right_line_mask_img)

        self.get_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        return   

    def pub_cv_img(self, cv_img):
        cv_img_copy = cv_img
        cv_img_copy_msg = self.bridge.cv2_to_imgmsg(cv_img_copy, encoding="bgr8")
        cv_img_copy_msg.header.stamp = rospy.Time.now()
        self.cv_image_pub.publish(cv_img_copy_msg)
        return       

    def pub_mask_img(self, mask_img):
        mask_img_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="passthrough")
        mask_img_msg.header.stamp = rospy.Time.now()
        self.mask_image_pub.publish(mask_img_msg)
        return

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

        self.center_x = config.center_x
        self.center_y = config.center_y
        
        return config

if __name__ == '__main__':
    N = RobotVision()
    rospy.spin()  