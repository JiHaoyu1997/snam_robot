#!/usr/bin/env python3

import rospy
from typing import List

from robot.robot import robot_dict, find_id_by_robot_name, RobotInfo, RobotMotion
from robot.decision_model import FIFOModel, GridBasedModel, GridBasedOptimalModel, VSCSModel

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

from vpa_robot.msg import RobotInfo as RobotInfoMsg
from vpa_robot.msg import InterInfo as InterInfoMsg
from vpa_robot.msg import KinematicDataArray
from vpa_robot_interface.msg import WheelsEncoder
from vpa_robot.srv import InterMng, InterMngRequest, InterMngResponse
from vpa_robot.srv import NewRoute, NewRouteRequest, NewRouteResponse
from vpa_robot.srv import ReadySignal, ReadySignalResponse

class InterInfo:
    def __init__(self, inter_id=0):
        self.inter_id = inter_id                # 交叉路口ID
        self.robot_id_list = []                 # 机器人ID列表
        self.robot_info: List[RobotInfo] = []   # RobotInfo实例列表


class RobotDecision:
    # Initialization
    def __init__(self) -> None:
        # Initialize ROS node
        rospy.init_node('decision_maker')
        rospy.on_shutdown(self.shutdown_handler)
        self.node_name = 'robot_decision' 

        self.robot_name = rospy.get_param('~robot_name', 'db19')
        print(self.robot_name)
        self.robot_id = find_id_by_robot_name(robot_name=self.robot_name)
        self.robot_info = RobotInfo(name=self.robot_name, robot_id=self.robot_id)
        self.decision_model = GridBasedModel(robot_id=self.robot_id)
        self.robot_motion_controller = RobotMotion(name=self.robot_name, robot_id=self.robot_id) 

        # Virtual Spring Control System
        self.vscs_model = VSCSModel(robot_id=self.robot_id)

        # Initialize route and intersection information
        self.curr_route = [0, 0, 0]
        self.local_inter_id = 0
        self.local_inter_info = InterInfo()
        self.local_inter_info_topic = f'/inter_info/{self.local_inter_id}'

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.robot_info_pub = rospy.Publisher('robot_info', RobotInfoMsg, queue_size=1)

        # Subscribers
        self.cmd_vel_from_img_sub = rospy.Subscriber('cmd_vel_from_img', Twist, self.cmd_vel_from_img_cb)
        self.kinematic_info_sub = rospy.Subscriber('/kinematic_info', KinematicDataArray, self.kinematic_info_cb)
        self.wheel_omega_sub = rospy.Subscriber('wheel_omega', WheelsEncoder, self.wheel_omega_cb)
        self.inter_info_sub = rospy.Subscriber(self.local_inter_info_topic, InterInfoMsg, self.inter_info_cb)
        self.shutdown_sub = rospy.Subscriber("robot_interface_shutdown", Bool, self.signal_shutdown)
        self.local_brake_sub = rospy.Subscriber("local_brake", Bool, self.local_brake_sub_cb)
        self.global_brake_sub = rospy.Subscriber("/global_brake", Bool, self.global_brake_sub_cb)
        
        # Servers
        self.update_route_server = rospy.Service('update_route_srv', NewRoute, self.update_route_cb)
        self.enter_conflict_server = rospy.Service('enter_conflict_srv', Trigger, self.enter_conflict_cb)

        # Clients
        self.inter_mng_client = rospy.ServiceProxy('/inter_mng_srv', InterMng)

        # Init
        self.send_ready_signal()

        self.timer = rospy.Timer(rospy.Duration(1 / 40), self.pub_robot_info)
    
    # Methods
    def send_ready_signal(self):
        """
        Send ready_signal to task_manager.
        """
        rospy.wait_for_service('ready_signal')
        try:
            ready_signal_client = rospy.ServiceProxy('ready_signal', ReadySignal)
            response: ReadySignalResponse = ready_signal_client.call(self.node_name)
            if response.success:
                rospy.loginfo(f"{self.robot_name}: Decision Maker is Online")
                self.curr_route = [6, 6, 2]   
                self.local_inter_id = 6
                self.robot_info.robot_route = self.curr_route
            return
        except rospy.ServiceException as e:
            rospy.logerr(f"service call failed: {e}")
            return

    def update_route_cb(self, req: NewRouteRequest):
        """
        Update curr_route.
        """
        new_route = [route for route in req.new_route]

        # Abnormal Detection
        if len(new_route) != 3:
            rospy.logerr("Route message does not contain 3 elements.")
            return NewRouteResponse(success=False, message='Update Error') 
        
        # Loop Init
        if new_route == [6, 6, 2]:
            self.curr_route = [6, 6, 2]
            self.robot_info.robot_route = self.curr_route
            return NewRouteResponse(success=True, message=f"{self.robot_name} start new travel")
        
        # Condition Match             
        if self.curr_route[1] == new_route[0] and self.curr_route[2] == new_route[1]:
            # get time
            now_time = round(rospy.get_time(), 5)
            travel_time = now_time - self.robot_info.robot_enter_lane_time
            rospy.loginfo(f"{self.robot_name} travel time in Inter{self.curr_route[1]}: {travel_time}")
            
            if new_route[1] == 3:
                self.robot_info.record_curr_state()
                self.vscs_model.generate_pass_cp_flag_dict(new_route)
                if self.departure_time:
                    travel_total_time = now_time - self.departure_time
                    rospy.loginfo(f"{self.robot_name} travel total time until now: {travel_total_time}")

            self.robot_info.robot_exit_time = now_time
            self.robot_info.robot_enter_lane_time = now_time

            # update local info
            self.curr_route = new_route

            # update pub info
            self.robot_info.robot_route = self.curr_route
            self.robot_info.robot_p = 0.0
            self.robot_info.robot_enter_conflict = False

            # update decision flag
            self.decision_model.want_to_enter_conflict = False
            self.decision_model.enter_permission = False
            self.decision_model.curr_route = tuple(new_route)

            # update global info
            self.update_global_inter_info(new_route=new_route)

            # update inter_x info sub 
            self.update_inter_sub(new_route=new_route)

            # reset segment travel distance
            self.robot_motion_controller.total_distance_apriltag = 0.0

            # response
            if new_route[1] == 6 and new_route[2] == 6:
                return NewRouteResponse(success=True, message=f"{self.robot_name} travel end")
            else:
                return NewRouteResponse(success=True, message=f"{self.robot_name} updated new inter info")
        else:
            rospy.logerr(f"Route update sequence mismatch: {self.curr_route} and {new_route}.") 
            return      

    def update_global_inter_info(self, new_route):
        """
        Request to update global inter_info.
        """
        rospy.wait_for_service('/inter_mng_srv')
        try:
            req = InterMngRequest()
            req.header.stamp = rospy.Time.now()
            req.robot_id = self.robot_id
            req.last_inter_id = new_route[0]
            req.curr_inter_id = new_route[1]
            resp: InterMngResponse = self.inter_mng_client.call(req)
            if resp.success:
                rospy.loginfo(f'{self.robot_name} - {resp.message}')
            else:
                rospy.logwarn(f'{self.robot_name} - Service response not successful: {resp.message}')
            return
        except rospy.ServiceException as e:
            rospy.logerr(f'{self.robot_name}: Service call failed - {e}')
            return

    def update_inter_sub(self, new_route):
        """
        Update local subscribed inter_info/x.
        """

        if hasattr(self, 'inter_info_sub'):
            self.inter_info_sub.unregister()
        
        self.local_inter_id = new_route[1]
        if self.local_inter_id == 6:
            rospy.loginfo(f"{self.robot_name} drives back to buffer area")
            return
        
        self.local_inter_info_topic = f'/inter_info/{self.local_inter_id}'
        self.inter_info_sub = rospy.Subscriber(self.local_inter_info_topic, InterInfoMsg, self.inter_info_cb)
        rospy.loginfo(f"{self.robot_name} updated inter_info_sub to {self.local_inter_info_topic}")
        return

    def inter_info_cb(self, inter_info_msg: InterInfoMsg):
        """
        Updates local intersection information.
        """
        # update_break_virtual_spring_flag_dict
        if self.local_inter_id == 3:
            if self.local_inter_info.robot_id_list != inter_info_msg.robot_id_list:
                rospy.logwarn("robot_id_list change")
                self.vscs_model.update_break_virtual_spring_flag_dict(new_robot_id_list=inter_info_msg.robot_id_list)

        # update inter_id
        self.local_inter_info.inter_id = inter_info_msg.inter_id

        # update robot_id_list
        self.local_inter_info.robot_id_list = inter_info_msg.robot_id_list

        # update robot_info_list
        robot_info: List[RobotInfoMsg] = inter_info_msg.robot_info
        self.local_inter_info.robot_info = [
            RobotInfo(
                name=info.robot_name,
                robot_id=info.robot_id,
                robot_route=info.robot_route,
                v=info.robot_v,
                p=info.robot_p,
                coordinate=info.robot_coordinate,
                enter_conflict=info.robot_enter_conflict,

                # time info
                robot_enter_lane_time=info.robot_enter_lane_time,
                robot_estimated_arrive_conflict_time=info.robot_estimated_arrive_conflict_time,
                robot_arrival_conflict_time=info.robot_arrival_conflict_time,
                robot_enter_conflict_time=info.robot_enter_conflict_time,
                robot_arrive_cp_time=info.robot_arrive_cp_time,
                robot_exit_time=info.robot_exit_time
            ) for info in robot_info
        ]

        return

    def cmd_vel_from_img_cb(self, msg: Twist):
        """
        Receive Twist messages from the vision system.
        """
        cmd_vel = self.make_decision(twist_from_img=msg)
        self.cmd_vel_pub.publish(cmd_vel)
        return

    def make_decision(self, twist_from_img: Twist) -> Twist:
        """
        Decision-making process based on the robot's current info.
        """

        # if self.decision_model.want_to_enter_conflict:
        #         if not self.decision_model.enter_permission:
        #             # print(self.local_inter_id)
        #             self.decision_model.enter_permission = self.decision_model.check_enter_permission(self.local_inter_info.robot_info)
        #             # print(self.decision_model.enter_permission)
        #             self.robot_info.robot_enter_conflict = self.decision_model.enter_permission
        #             # print(self.robot_info.robot_enter_conflict)
        #             # robot entet conflict
        #             if self.decision_model.enter_permission:
        #                 self.robot_info.robot_enter_conflict_time = rospy.get_time()
        #                 wait_time = self.robot_info.calc_wait_time()
        #                 rospy.loginfo(f"{self.robot_name} wait time in inter{self.curr_route[1]}: {wait_time}")
        # twist_from_decision = self.decision_model.decision_maker(twist_from_img)
        # return twist_from_decision
        
        if self.curr_route[1] != 3:
            if self.decision_model.want_to_enter_conflict:
                if not self.decision_model.enter_permission:
                    self.decision_model.enter_permission = self.decision_model.check_enter_permission(self.curr_route, self.local_inter_info.robot_info)
                    # print(self.decision_model.enter_permission)
                    self.robot_info.robot_enter_conflict = self.decision_model.enter_permission
                    # print(self.robot_info.robot_enter_conflict)
                    # robot entet conflict
                    if self.decision_model.enter_permission:
                        self.robot_info.robot_enter_conflict_time = rospy.get_time()
                        wait_time = self.robot_info.calc_wait_time()
                        rospy.loginfo(f"{self.robot_name} wait time in inter{self.curr_route[1]}: {wait_time}")
            twist_from_decision = self.decision_model.decision_maker(twist_from_img)
        else:
            robot_id_list = self.local_inter_info.robot_id_list
            robot_info_list = self.local_inter_info.robot_info
            twist_from_decision = self.vscs_model.calc_twist(twist_from_img, robot_id_list, robot_info_list)
        return twist_from_decision

    def enter_conflict_cb(self, req):
        """
        Update the robot's conflict zone entry state based on the received message.
        """
        # arrival at the conflict boundary line (red stop line)
        self.decision_model.want_to_enter_conflict = True

        # get time
        self.robot_info.robot_arrival_conflict_time = rospy.get_time()
        lane_time = self.robot_info.calc_lane_travel_time()
        # rospy.loginfo(f"{self.robot_name} lane travel time in route{self.curr_route} = {round(lane_time, 5)} | distance {round(self.robot_info.robot_p, 5)}")

        # respond
        response = TriggerResponse()
        response.success = True
        response.message = f"{self.robot_name} want to enter conflict zone"
        return response
    
    def kinematic_info_cb(self, kinematic_data_msg: KinematicDataArray):
        """
        Receive self kinematics data and update local info.
        """
        for data in kinematic_data_msg.data:
            if self.robot_id == data.robot_id:
                curr_pose = data.pose
                curr_vel = data.vel

                # update kinematic info
                self.robot_motion_controller.kinematic_recoder(pose=curr_pose, vel=curr_vel)

                # update local info
                self.robot_info.robot_v = curr_vel[0]
                self.robot_info.robot_p = self.robot_motion_controller.total_distance_apriltag
                self.robot_info.robot_coordinate = (curr_pose[1], curr_pose[2])

        return
    
    def wheel_omega_cb(self, wheel_omega_msg: WheelsEncoder):
        """
        Through wheel encoder to calc travel time.
        """
        pass
        # return self.robot_motion_controller.tick_recorder(msg=wheel_omega_msg)    
    
    def local_brake_sub_cb(self, msg: Bool):
        pass
    
    def global_brake_sub_cb(self, msg: Bool):
        if not msg.data:
            self.departure_time = round(rospy.get_time(), 5)

    def pub_robot_info(self, event):
        """
        Publisch self robot_info.
        """
        if self.curr_route == [6,6,2] or self.curr_route == [2,6,6]:
            est_time = 0.0
        elif self.decision_model.want_to_enter_conflict:
            est_time = 0.0
        else:
            est_time = 0.0
            # est_time = self.robot_motion_controller.calc_estimated_arrive_conflict_time(route=self.curr_route)

        self.robot_info.robot_estimated_arrive_conflict_time = est_time
        robot_info_msg = self.robot_info.to_robot_info_msg()
        self.robot_info_pub.publish(robot_info_msg)
        return
    
    def shutdown_handler(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        return

    def signal_shutdown(self, msg: Bool):
        if msg.data:
            rospy.signal_shutdown('decision maker node shutdown')
        return


if __name__ == '__main__':
    N = RobotDecision()
    rospy.spin()