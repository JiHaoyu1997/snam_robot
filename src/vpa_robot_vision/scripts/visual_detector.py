#! /usr/bin/env python3

import rospy

import threading
from enum import Enum


import cv2
from cv_bridge import CvBridge

from map import map
from hsv import hsv, search_pattern
from pid_controller import pid_controller

# Dynamic reconfiguration
from dynamic_reconfigure.server import Server
from vpa_robot_vision.cfg import color_hsvConfig

# Msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image 

from vpa_robot_decision.srv import NewRoute, NewRouteRequest, NewRouteResponse
from vpa_robot_task.srv import AssignRoute, AssignRouteRequest, AssignRouteResponse
from vpa_robot_task.srv import ReadySignal, ReadySignalResponse

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

        #
        self.node_name = 'robot_vision'

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

        # 
        self.status_flag_init()

        # init hsv color spaces for selecting 
        self.color_space_init()

        # Publishers
        self.cv_image_pub = rospy.Publisher('cv_image', Image, queue_size=1)
        self.mask_image_pub = rospy.Publisher('mask_image', Image, queue_size=1)
        self.cmd_vel_from_img_pub = rospy.Publisher('cmd_vel_from_img', Twist, queue_size=1)
 
        # Subscribers
        self.image_raw_sub = rospy.Subscriber("robot_cam/image_raw", Image, self.image_raw_sub_cb)

        # Servers
        self.srv_color = Server(color_hsvConfig, self.dynamic_reconfigure_callback_hsv)

        # Clients
        self.assign_route_client = rospy.ServiceProxy('assign_route_srv', AssignRoute)
        self.update_route_client = rospy.ServiceProxy('update_route_srv', NewRoute)

        self.send_ready_signal()
    
    # Methods
    def send_ready_signal(self):
        rospy.wait_for_service('ready_signal')
        try:
            ready_signal_client = rospy.ServiceProxy('ready_signal', ReadySignal)
            response: ReadySignalResponse = ready_signal_client.call(self.node_name)
            if response.success:
                rospy.loginfo(f"{self.robot_name}: Visual Detector is Online")
                self.curr_route = [6, 6, 2]

        except rospy.ServiceException as e:
            rospy.logerr(f"service call failed: {e}")

    def status_flag_init(self):
        # Current route (three intersections: last, current and next)
        self.curr_route = [0, 0, 0]

        # Current zone (default buffer area)
        self.current_zone = Zone.BUFFER_AREA

        # self.lane_direction = LaneDirection.RIGHT_HAND 

        # Intersection Boundary Line count
        self.cross_inter_boundary_line_count = 0

        # Boundary Crossing flag
        self.boundary_detect_req_lock = threading.Lock()

        # Enter Conflict Zone flag
        self.enter_conflict_zone = False

        # Stop Operation
        self.stop = False

        # 
        self.stop_timer = None

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
        self.next_action = 0        # the next action to perfrom 
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

        # 
        self.stop_line_hsv2 = hsv.HSV_RANGES['red2'] 

        # Buffer Line HSV - Pink
        self.buffer_line_hsv = hsv.HSV_RANGES['pink']

        # Overexposed_line_hsv White
        self.overexposed_line_hsv = hsv.HSV_RANGES['expos']

        # Ready Line HSV - Blue
        self.ready_line_hsv = hsv.HSV_RANGES['blue']

        # Intersection Boundary Line HSV - Green
        self.inter_boundary_line_hsv = hsv.HSV_RANGES['green']

        # guiding lines inside intersections
        self.thur_guide_hsv = hsv.HSV_RANGES['blue'] 
        self.left_guide_hsv = hsv.HSV_RANGES['purple']
        self.right_guide_hsv = hsv.HSV_RANGES['orange']
        self.inter_guide_line = [self.thur_guide_hsv, self.left_guide_hsv, self.right_guide_hsv]

        # 
        self._acc_aux_hsv = hsv.HSV_RANGES['red']       

    def image_raw_sub_cb(self, data: Image):
        """Step1 CONVERT RAW IMAGE TO HSV IMAGE"""
        cv_img, cv_hsv_img, acc_hsv_img = hsv.convert_raw_img_to_hsv_img(data=data, cv_bridge=self.cv_bridge)

        """Step2 Test Mode"""
        if self.test_mode != 'default':
            self.test_mode_func(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
            return
        
        """Step3 Init"""
        if self.curr_route == [0, 0, 0]:
            return
        else:
            rospy.loginfo_once("Start Tracking")

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
        dis2bound = search_pattern.search_line(cv_hsv_img, self.inter_boundary_line_hsv, top_line=100)
        cross_inter_boundary = dis2bound > 25

        if cross_inter_boundary:
            self.cross_inter_boundary_line_count += 1
            if self.cross_inter_boundary_line_count >= 2:
                if self.boundary_detect_req_lock.acquire(blocking=False):
                    try:
                        self.enter_conflict_zone = False
                        new_route = self.req_new_route()
                        new_route = [route for route in new_route]
                        self.curr_route = new_route
                        threading.Thread(target=self.req_update_new_route, args=(new_route,)).start()
                    finally:
                        pass
        else:
            self.cross_inter_boundary_line_count = 0
            if self.boundary_detect_req_lock.locked():
                self.boundary_detect_req_lock.release()

        return

    def req_new_route(self):
        try:
            route = [0, 0, 0]
            req = AssignRouteRequest()
            req.last_inter_id = self.curr_route[1]
            req.next_inter_id = self.curr_route[2]
            resp: AssignRouteResponse = self.assign_route_client.call(req)
            rospy.loginfo(f'{self.robot_name} cross the boundary line between inter{self.curr_route[1]} and inter{self.curr_route[2]}')
            return resp.route if resp.route else route
        except rospy.ServiceException as e:
            rospy.logerr('%s: Request New Route Service Call Failed: %s', self.robot_name, e)
            return [0, 0, 0]

    def req_update_new_route(self, new_route):
        rospy.wait_for_service('update_route_srv')
        try:
            req = NewRouteRequest(new_route=new_route)
            resp: NewRouteResponse = self.update_route_client.call(req)
            if resp.success:
                rospy.loginfo(resp.message)
            else:
                rospy.logerr(resp.message)
        except rospy.ServiceException as e:
            rospy.logerr('%s: Request update inter info service call failed: %s', self.robot_name, e)

    def find_and_draw_target(self, cv_img, cv_hsv_img):
        rospy.loginfo(f"current route is {self.curr_route}")
        target_x = 0
        # --- BUFFER AREA ---
        
        # NO TASK ==> STOP AT READY_LINE 
        if self.curr_route == [2, 6, 6]:
            self.current_zone = Zone.BUFFER_AREA
            dis2ready = search_pattern.search_line(cv_hsv_img, self.ready_line_hsv)
            if dis2ready > 25:
                self.stop = True
                return target_x, cv_img
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
            dis2red = search_pattern.search_line(cv_hsv_img, self.stop_line_hsv)
            if dis2red > 25:
                self.enter_conflict_zone = True

            if not self.enter_conflict_zone:         
                try:                 
                    target_x, cv_img = self.find_target_from_buffer_line(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
                except Exception as e:
                    print(e)

            else:
                rospy.loginfo(f"Enter Conflict Zone")
                self.next_action = map.local_mapper(last=self.curr_route[0], current=self.curr_route[1], next=self.curr_route[2])
                rospy.loginfo(f"Next Action is {self.action_dic[self.next_action]}")               
                target_x, cv_img = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, action=self.next_action)

        # GENERAL ROUTE
        else:
            target_x, cv_img = self.cross_intersection(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
        
        return target_x, cv_img
    
    def cross_intersection(self, cv_img, cv_hsv_img):
        self.current_zone = Zone.INTERSECTION
        # check if conflict zone
        self.detect_conflict_boundary_line(cv_hsv_img=cv_hsv_img)
        if not self.enter_conflict_zone:
            # lane
            target_x, cv_img = self.find_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
        else:
            # conflict zone
            rospy.loginfo(f"Enter Conflict Zone")
            self.next_action = map.local_mapper(last=self.curr_route[0], current=self.curr_route[1], next=self.curr_route[2])
            rospy.loginfo(f"Next Action is {self.action_dic[self.next_action]}") 
            target_x, cv_img = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, action=self.next_action)

        return target_x, cv_img

    def detect_conflict_boundary_line(self, cv_hsv_img):
        dis2conflict = search_pattern.search_stop_line(cv_hsv_img, self.stop_line_hsv, self.stop_line_hsv2)
        # print(dis2conflict)
        if dis2conflict > 30:
            self.enter_conflict_zone = True
    
    def find_target_from_buffer_line(self, cv_img, cv_hsv_img):
        buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(cv_hsv_img=cv_hsv_img, buffer_line_hsv=self.buffer_line_hsv)

        if buffer_line_x == None:
            buffer_line_x, buffer_line_y = search_pattern.search_buffer_line(cv_hsv_img=cv_hsv_img, buffer_line_hsv=self.overexposed_line_hsv)

        if buffer_line_x == None:
            buffer_line_x = int(cv_hsv_img.shape[1] / 2)

        if buffer_line_y == None:
            buffer_line_y = int(cv_hsv_img.shape[0] / 2)
        
        target_x = buffer_line_x         
        cv2.circle(cv_img, (buffer_line_x, buffer_line_y), 5, (255, 100, 0), 5)       
        return target_x, cv_img

    def find_target_to_cross_lane(self, cv_img, cv_hsv_img):
        target_x = search_pattern.search_lane_center(self.center_line_hsv, self.side_line_hsv, cv_hsv_img, is_yellow_left=True)
        if target_x == None:
            target_x = self.image_width / 2
        cv2.circle(cv_img, (int(target_x), int(cv_hsv_img.shape[0]/2)), 5, (0, 255, 0), 5)
        return target_x, cv_img
    
    def find_target_to_cross_conflict(self, cv_img, cv_hsv_img, action):
        hsv_space: hsv.HSVSpace = self.inter_guide_line[action]
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

        elif self.test_mode == 'buffer':
            self.go_thur_buffer(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        elif self.test_mode == 'lane':
            self.go_thur_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        elif self.test_mode == 'rightcircle':
            self.go_thur_right_circle(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        elif self.test_mode == 'sturn':
            self.go_sturn(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

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
        mask_img = self.center_line_hsv.apply_mask(cv_hsv_img)
        _line_center1 = search_pattern._search_lane_linecenter(mask_img, -20, 50, 10, int(cv_hsv_img.shape[0]/2), 0, int(cv_hsv_img.shape[1]))
        self.pub_mask_img(mask_img=mask_img)

        # self.find_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)

        return
    
    def go_thur_buffer(self, cv_img, cv_hsv_img):
        dis2orange = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=hsv.HSV_RANGES['orange'])
        if dis2orange > 30:
            self.stop = True
            rospy.loginfo("STOP")
        
        dis2green = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.inter_boundary_line_hsv)
        if dis2green > 30:
            self.stop = False
            rospy.loginfo("Start")
        
        target_x, cv_img = self.find_target_from_buffer_line(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
        v_x, omega_z = self.calculate_velocity(target_x=target_x)
        self.pub_cmd_vel_from_img(v_x, omega_z)  

        mask_img = self.buffer_line_hsv.apply_mask(cv_hsv_img)
           
        self.pub_cv_img(cv_img=cv_img)
        self.pub_mask_img(mask_img=mask_img)
        
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
        if dis2green > 30 and self.stop_timer is None:
            rospy.loginfo("Start")
            self.stop_timer = rospy.Timer(rospy.Duration(1), self.stop_cb, oneshot=True)
       
        target_x, _ = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, action=action)
        # print(target_x)
        v_x, omega_z = self.calculate_velocity(target_x=target_x)
        self.pub_cmd_vel_from_img(v_x, omega_z)  
        mask_img = hsv_space.apply_mask(cv_hsv_img)

        start_point = (0, 110)
        end_point = (cv_img.shape[1] - 1, 110)
        color = (0, 0, 255)
        thickness = 1
        cv2.line(cv_img, start_point, end_point, color, thickness)

        start_point2 = (0, 130)
        end_point2 = (cv_img.shape[1] - 1, 130)
        color2 = (0, 255, 0)
        thickness2 = 1
        cv2.line(cv_img, start_point2, end_point2, color2, thickness2)

        start_point = (40, 1)
        end_point = (280, 178)
        color = (255, 0, 255)
        thickness = 1
        cv2.rectangle(cv_img, start_point, end_point, color, thickness)
        
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
            target_x, cv_img = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, action=action)
            v_x, omega_z = self.calculate_velocity(target_x=target_x)
            mask_img = self.right_guide_hsv.apply_mask(cv_hsv_img)

        self.pub_cmd_vel_from_img(v_x, omega_z)             
        self.pub_cv_img(cv_img=cv_img)
        self.pub_mask_img(mask_img=mask_img)     

        return
    
    def go_sturn(self, cv_img, cv_hsv_img):
        # 检测红色区域
        dis2red = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.stop_line_hsv)
        if dis2red > 30:
            self.in_lane = False
            rospy.loginfo("Conflict Zone")

            # 仅当 action 尚未更新时增加
            if not hasattr(self, 'action_updated') or not self.action_updated:
                self.next_action += 1
                self.action_updated = True  # 设置标志位，防止重复增加

        # 
        if self.next_action == 3:
            self.stop = True
            self.pub_cmd_vel_from_img(0, 0)
            return

        # 检测绿色区域
        dis2green = search_pattern.search_line(hsv_image=cv_hsv_img, hsv_space=self.inter_boundary_line_hsv)
        if dis2green > 30:
            self.in_lane = True
            self.action_updated = False  # 重置标志位，允许在检测到下一个冲突区域时再增加 action
            rospy.loginfo("Lane Zone")

        # 根据位置选择不同的目标处理方式
        if self.in_lane:
            target_x, cv_img = self.find_target_to_cross_lane(cv_img=cv_img, cv_hsv_img=cv_hsv_img)
            v_x, omega_z = self.calculate_velocity(target_x=target_x)

            hsv_image1 = cv_hsv_img
            hsv_image2 = cv_hsv_img
            mask1 = self.center_line_hsv.apply_mask(hsv_image1)
            mask2 = self.side_line_hsv.apply_mask(hsv_image2)
            mask_img = mask1 + mask2
        else:           
            target_x, cv_img = self.find_target_to_cross_conflict(cv_img=cv_img, cv_hsv_img=cv_hsv_img, action=self.next_action)
            if self. next_action == 2:
                print(target_x)
            v_x, omega_z = self.calculate_velocity(target_x=target_x)
            mask_img = self.inter_guide_line[self.next_action].apply_mask(cv_hsv_img)

        # 发布速度和图像
        self.pub_cmd_vel_from_img(v_x, omega_z)             
        self.pub_cv_img(cv_img=cv_img)
        self.pub_mask_img(mask_img=mask_img)     

        return
    
    def stop_cb(self, event):
        self.stop = True
        rospy.loginfo("STOP")
        self.stop_timer = None
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