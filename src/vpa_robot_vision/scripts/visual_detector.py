#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from enum import Enum

from map import map
from hsv import hsv, search_pattern
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
        self.test_mode = rospy.get_param('~test_mode', 'default')

        # Image Size
        self.image_width = rospy.get_param('~image_width', 320)
        self.image_height = rospy.get_param('~image_height', 240)

        # Default Velocity Setup
        self.v_set = VelocitySet()

        # std_msgs img ==> ros img
        self.cv_bridge = CvBridge()

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

        # 
        self.in_lane = True

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
        self.center_line_hsv = hsv.HSVSpace(
            h_u=int(rospy.get_param('~h_upper_1', 0)),
            h_l=int(rospy.get_param('~h_lower_1', 0)),
            s_u=int(rospy.get_param('~s_upper_1', 0)),
            s_l=int(rospy.get_param('~s_lower_1', 0)),
            v_u=int(rospy.get_param('~v_upper_1', 0)),
            v_l=int(rospy.get_param('~v_lower_1', 0))
        ) 

        # HSV space for White (side lane line)
        self.side_line_hsv = hsv.HSVSpace(
            h_u=int(rospy.get_param('~h_upper_2', 0)),
            h_l=int(rospy.get_param('~h_lower_2', 0)),
            s_u=int(rospy.get_param('~s_upper_2', 0)),
            s_l=int(rospy.get_param('~s_lower_2', 0)),
            v_u=int(rospy.get_param('~v_upper_2', 0)),
            v_l=int(rospy.get_param('~v_lower_2', 0))
        ) 

        # HSV space for Red (stop line)
        self.stop_line_hsv = hsv.HSVSpace(
            h_u=int(rospy.get_param('~h_upper_s', 0)),
            h_l=int(rospy.get_param('~h_lower_s', 0)),
            s_u=int(rospy.get_param('~s_upper_s', 0)),
            s_l=int(rospy.get_param('~s_lower_s', 0)),
            v_u=int(rospy.get_param('~v_upper_s', 0)),            
            v_l=int(rospy.get_param('~v_lower_s', 0))
        )

        # Buffer Line HSV - Pink
        self.buffer_line_hsv = hsv.HSVSpace(h_u=190, h_l=150, s_u=255, s_l=60, v_u=255, v_l=185)
        self.overexposed_buffer_line_hsv = hsv.HSVSpace(h_u=20, h_l=0, s_u=55, s_l=45, v_u=255, v_l=205)

        # Ready Line HSV - Yellow
        self.ready_line_hsv = self.side_line_hsv

        # Intersection Boundary Line HSV - Green
        self.inter_boundary_line_hsv = hsv.HSVSpace(h_u=95, h_l=65, s_u=255, s_l=150, v_u=200, v_l=70)

        # guiding lines inside intersections - no dynamic reconfigure
        self.thur_guide_hsv = hsv.HSVSpace(h_u=125, h_l=95, s_u=255, s_l=180, v_u=255, v_l=120)  
        self.left_guide_hsv = hsv.HSVSpace(h_u=175, h_l=145, s_u= 200, s_l=50, v_u=255, v_l=100)
        self.right_guide_hsv = hsv.HSVSpace(h_u=25, h_l=4, s_u= 255, s_l=100, v_u=255, v_l=150)
        self.inter_guide_line = [self.thur_guide_hsv, self.left_guide_hsv, self.right_guide_hsv]

        # 
        self._acc_aux_hsv = hsv.HSVSpace(150, 110, 180, 100, 255, 120)       

    def curr_route_sub_cb(self, route_msg: Int8MultiArray):
        if route_msg:
            self.curr_route = [route for route in route_msg.data]
        else:
            self.curr_route = [0 , 0, 0]

    def image_raw_sub_cb(self, data: Image):
        """Step1 CONVERT RAW IMAGE TO HSV IMAGE"""
        cv_img, cv_hsv_img, acc_hsv_img = hsv.convert_raw_img_to_hsv_img(data=data, cv_bridge=self.cv_bridge)

        """Step2 Test Mode"""
        if self.test_mode != 'default':
            self.test_mode_func(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
            return
        
        """Step3 Init"""
        if self.curr_route == [0, 0, 0]:
            rospy.loginfo("Not Start Tracking")
            return

        """Step4 BOUNDARY LINE DETECTOR"""
        self.detect_inter_boundary_line(cv_hsv_img=cv_hsv_img) 

        """Step5 FROM HSV IMAGE TO TARGET COORDINATE"""
        target_x, result_cv_img = self.find_and_draw_target(cv_img=cv_img, cv_hsv_img=cv_hsv_img)  
                
        """Step6 FROM TARGET COORDINATE TO TWIST"""
        v_x, omega_z = self.calculate_velocity(target_x=target_x)
           
        """Step7 PUB TWIST TO DECISION MAKER"""
        # since the cmd_vel is sending on a higher frequency than ACC msg, we estimate the distance to further avoid collsions
        self.pub_cmd_vel_from_img(v_x, omega_z)

        return self.pub_cv_img(cv_img=result_cv_img)

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

    def find_and_draw_target(self, cv_img, cv_hsv_img):
        # --- BUFFER AREA ---
        
        # NO TASK ==> STOP AT READY_LINE 
        if self.curr_route == [2, 6, 6]:
            self.current_zone = Zone.BUFFER_AREA
            dis2ready = search_pattern.search_line(cv_hsv_img, self.ready_line_hsv)
            if dis2ready > 25:
                self.stop = True
                return
            else:
                target_x, cv_img = self.find_target_from_buffer_line(cv_img=cv_img, cv_hsv_img=cv_hsv_img)         
            
        # INIT TASK ==> FROM BUFFER TO INTERSECTION          
        elif self.curr_route == [6, 6, 2]:
            self.current_zone = Zone.BUFFER_AREA
            target_x, cv_img = self.find_target_from_buffer_line(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        # --- INTERSECTION AREA ---

        # DEFAULT FIRST ROUTE [6, 2, X]
        elif self.curr_route[0] == 6 and self.curr_route[1] == 2:
            self.current_zone = Zone.INTERSECTION
            dis2inside = search_pattern.search_line(cv_hsv_img, self.side_line_hsv)
            if dis2inside > 25:
                self.enter_conflict_zone = True

            if not self.enter_conflict_zone:                          
                #
                target_x, cv_img = self.find_target_from_buffer_line(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
            else: 
                # 
                target_x, cv_img = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        # GENERAL ROUTE
        else:
            target_x, cv_img = self.cross_intersection(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
        
        return target_x, cv_img
    
    def cross_intersection(self, cv_img, cv_hsv_img):
        self.current_zone = Zone.INTERSECTION
        rospy.loginfo(f"current route is {self.curr_route}")
        # check if conflict zone
        self.detect_conflict_boundary_line(cv_hsv_img=cv_hsv_img)
        # lane
        if not self.enter_conflict_zone:
            target_x, cv_img = self.find_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
        # conflict zone
        else:
            rospy.loginfo(f"Enter Conflict Zone")
            self.next_action = map.local_mapper(last=self.curr_route[0], current=self.curr_route[1], next=self.curr_route[2])
            rospy.loginfo(f"Next Action is {self.action_dic[self.next_action]}")
            hsv_space = self.inter_guide_line[self.next_action]
            target_x, cv_img = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, hsv_space=hsv_space, action=self.next_action)

        return target_x, cv_img

    def detect_conflict_boundary_line(self, cv_hsv_img):
        dis2conflict = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.stop_line_hsv)
        # print(dis2conflict)
        if dis2conflict > 30:
            self.enter_conflict_zone = True
    
    def find_target_from_buffer_line(self, cv_img, cv_hsv_img):
        buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(cv_hsv_img=cv_hsv_img, buffer_line_hsv=self.buffer_line_hsv)

        if buffer_line_x == None:
            buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(cv_hsv_img=cv_hsv_img, buffer_line_hsv=self.overexposed_buffer_line_hsv)
        
        if buffer_line_x == None:
            buffer_line_x = self.image_width * 2 / 5
        
        target_x = buffer_line_x         
        cv2.circle(cv_img, (int(buffer_line_x), buffer_line_y), 5, (255, 100, 0), 5)       
        return target_x, cv_img

    def find_target_to_cross_lane(self, cv_img, cv_hsv_img):
        target_x = search_pattern.search_lane_center(self.center_line_hsv, self.side_line_hsv, cv_hsv_img, is_yellow_left=True)
        if target_x == None:
            target_x = self.image_width / 2
        cv2.circle(cv_img, (int(target_x), int(cv_hsv_img.shape[0]/2)), 5, (0, 255, 0), 5)
        return target_x, cv_img
    
    def find_target_to_cross_conflict(self, cv_img, cv_hsv_img, hsv_space, action):
        target_x = search_pattern.search_inter_guide_line2(hsv_space, cv_hsv_img, action)
        if target_x == None:
            target_x = self.image_width / 2
        cv2.circle(cv_img, (int(target_x), int(cv_hsv_img.shape[0]/2)), 5, (255, 255, 0), 5)
        return target_x, cv_img
    
    def calculate_velocity(self, target_x):
        if self.stop:
            v_x = 0
            omega_z = 0
        
        elif self.current_zone == Zone.BUFFER_AREA:
            v_x, omega_z = pid_controller.bufffer_pi_control(
                int(self.image_width / 2), 
                target_x, 
                self.v_set.v_f_buffer, 
                self.v_set.v_s_buffer) 
               
        elif self.current_zone == Zone.INTERSECTION:
            v_x, omega_z = pid_controller.inter_pi_control(
                int(self.image_width / 2), 
                target_x, 
                self.v_set.v_f_inter, 
                self.v_set.v_s_inter)
            
        return v_x, omega_z

    def pub_cmd_vel_from_img(self, v_x, omega_z):
        if self.acc_mode:
            v_factor = 1
        else:
            v_factor = 1

        cmd_msg = Twist()
        cmd_msg.linear.x = v_x * v_factor
        cmd_msg.angular.z = omega_z * v_factor
        self.cmd_vel_from_img_pub.publish(cmd_msg)

        return

    def test_mode_func(self, cv_img, cv_hsv_img):
        if self.test_mode == 'hsv':
            self.find_hsv(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        elif self.test_mode == 'lane':
            self.go_thur_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        elif self.test_mode == 'rightcircle':
            self.go_thur_right_circle(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        else:
            self.go_thur_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        return
    
    def find_hsv(self, cv_img, cv_hsv_img):
        # Calculate the center coordinates as integers
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

        self.find_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        return
    
    def go_thur_lane(self, cv_img, cv_hsv_img):
        dis2red = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.stop_line_hsv)
        if dis2red > 30:
            self.stop = True
            rospy.loginfo("STOP")

        dis2green = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.inter_boundary_line_hsv)
        if dis2green > 30:
            self.stop = False
            rospy.loginfo("Start")

        target_x, _ = self.find_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
        v_x, omega_z = self.calculate_velocity(target_x=target_x)
        self.pub_cmd_vel_from_img(v_x, omega_z)  

        hsv_image1 = cv_hsv_img
        hsv_image2 = cv_hsv_img
        mask1 = self.center_line_hsv.apply_mask(hsv_image1)
        mask2 = self.side_line_hsv.apply_mask(hsv_image2)
        mask_img = mask1 + mask2
        
        self.pub_cv_img(cv_img=cv_img)
        self.pub_mask_img(mask_img=mask_img)
        
        return 
    
    def go_thur_conflict(self, cv_img, cv_hsv_img):
        action = {'left': 1, 'right': 2, 'thur': 0}.get(self.test_mode, None)
        hsv_space: hsv.HSVSpace = self.inter_guide_line[action]

        dis2red = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.stop_line_hsv)
        if dis2red > 30:
            self.stop = False
            rospy.loginfo("Start")

        dis2green = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.inter_boundary_line_hsv)
        if dis2green > 30:
            self.stop = True
            rospy.loginfo("STOP")         
       
        target_x, _ = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, hsv_space=hsv_space, action=action)
        print(target_x)
        v_x, omega_z = self.calculate_velocity(target_x=target_x)
        self.pub_cmd_vel_from_img(v_x, omega_z)  
        mask_img = hsv_space.apply_mask(cv_hsv_img)
        
        self.pub_cv_img(cv_img=cv_img)
        self.pub_mask_img(mask_img=mask_img)     

        return
    
    def go_thur_right_circle(self, cv_img, cv_hsv_img):
        action = 2

        dis2red = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.stop_line_hsv)
        if dis2red > 30:
            self.in_lane = False
            rospy.loginfo("Conflict Zone")

        dis2green = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.inter_boundary_line_hsv)
        if dis2green > 30:
            self.in_lane = True
            rospy.loginfo("Lane Zone")      

        if self.in_lane:
            target_x, cv_img = self.find_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
            v_x, omega_z = self.calculate_velocity(target_x=target_x)

            hsv_image1 = cv_hsv_img
            hsv_image2 = cv_hsv_img
            mask1 = self.center_line_hsv.apply_mask(hsv_image1)
            mask2 = self.side_line_hsv.apply_mask(hsv_image2)
            mask_img = mask1 + mask2

        else:           
            target_x, cv_img = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, hsv_space=self.right_guide_hsv, action=action)
            v_x, omega_z = self.calculate_velocity(target_x=target_x)
            mask_img = self.right_guide_hsv.apply_mask(cv_hsv_img)

        self.pub_cmd_vel_from_img(v_x, omega_z)             
        self.pub_cv_img(cv_img=cv_img)
        self.pub_mask_img(mask_img=mask_img)     

        return
    
    def pub_cv_img(self, cv_img):
        cv_img_copy = cv_img
        cv_img_copy_msg = self.cv_bridge.cv2_to_imgmsg(cv_img_copy, encoding="bgr8")
        cv_img_copy_msg.header.stamp = rospy.Time.now()
        self.cv_image_pub.publish(cv_img_copy_msg)
        return       

    def pub_mask_img(self, mask_img):
        mask_img_msg = self.cv_bridge.cv2_to_imgmsg(mask_img, encoding="passthrough")
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