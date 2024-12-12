#!/usr/bin/env python3

import rospy
from typing import List

from robot.robot import find_id_by_robot_name, RobotMotion

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from vpa_robot.msg import RobotInfo as RobotInfoMsg
from vpa_robot.msg import InterInfo as InterInfoMsg
from vpa_robot.msg import KinematicDataArray
from vpa_robot_interface.msg import WheelsEncoder
from vpa_robot.srv import InterMng, InterMngRequest, InterMngResponse
from vpa_robot.srv import NewRoute, NewRouteRequest, NewRouteResponse
from vpa_robot.srv import ReadySignal, ReadySignalResponse

class RobotInfo:
    def __init__(self, name="", robot_id=0, robot_route=[0, 0, 0], v=0.0, p=0.0, coordinate=(0.0, 0.0), enter_time=0.0, enter_conflict=False, arrive_cp_time=0.0, exit_time=0.0):
        self.robot_name = name
        self.robot_id = robot_id
        self.robot_route = robot_route
        self.robot_v = v
        self.robot_p = p
        self.robot_coordinate = coordinate
        self.robot_enter_time = enter_time
        self.robot_enter_conflict = enter_conflict
        self.robot_arrive_cp_time = arrive_cp_time
        self.robot_exit_time = exit_time

    def to_robot_info_msg(self):
        """
        Annotation
        """
        msg = RobotInfoMsg()
        msg.robot_name = self.robot_name
        msg.robot_id = self.robot_id
        msg.robot_route = self.robot_route
        msg.robot_v = self.robot_v
        msg.robot_p = self.robot_p
        msg.robot_coordinate = self.robot_coordinate
        msg.robot_enter_time = self.robot_enter_time
        msg.robot_enter_conflict = self.robot_enter_conflict
        msg.robot_arrive_cp_time = self.robot_arrive_cp_time
        msg.robot_exit_time = self.robot_exit_time
        return msg


class InterInfo:
    def __init__(self, inter_id=0):
        self.inter_id = inter_id                # 交叉路口ID
        self.robot_id_list = []                 # 机器人ID列表
        self.robot_info: List[RobotInfo] = []   # RobotInfo实例列表


class FCFSModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.want_to_enter_conflict = False
        self.enter_permission = False

    def decision_maker(self, twist_from_img: Twist):
        """
        Decision logic:
        """
        # If not yet in the conflict zone, return the original control command
        if not self.want_to_enter_conflict:
            return twist_from_img
        
        if not self.enter_permission:
            return Twist()
        else:
            return twist_from_img

    def check_enter_permission(self, robot_info_list: List[RobotInfo]):
        """
        Check the status of robots in the queue:
        """
        for robot_info in robot_info_list:
            # skip self
            if robot_info.robot_id == self.robot_id:
                continue

            # A robot has already entered the conflict zone; this robot cannot pass
            if robot_info.robot_enter_conflict:
                return False 
            
        # No robots have entered the conflict zone; this robot can pass
        return True 
    

class CBAAandDMPC:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.enter_conflict_zone = False

    def get_priority_by_CBAA(self):
        pass

    def get_twist_by_DMPC(self):
        pass

    def decision_maker(self, twist_from_img, robot_info):
        priority = self.get_priority_by_CBAA()
        twist = self.get_twist_by_DMPC()
        return twist       


class RobotDecision:
    # Initialization
    def __init__(self) -> None:
        # Initialize ROS node
        rospy.init_node('decision_maker')
        rospy.on_shutdown(self.shutdown_handler)
        self.node_name = 'robot_decision' 

        # Robot name
        self.robot_name = rospy.get_param('~robot_name', 'db19')
        self.robot_id = find_id_by_robot_name(robot_name=self.robot_name)
        self.robot_info = RobotInfo(name=self.robot_name, robot_id=self.robot_id)
        self.decision_model = FCFSModel(robot_id=self.robot_id)
        self.robot_motion_controller = RobotMotion(name=self.robot_name, robot_id=self.robot_id) 

        # Initialize route and intersection information
        self.curr_route = [0, 0, 0]
        self.local_inter_id = 0
        self.local_inter_info = InterInfo()
        self.local_inter_info_topic = f'/inter_info/{self.local_inter_id}'

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.robot_info_pub = rospy.Publisher('robot_info', RobotInfoMsg, queue_size=1)

        # Subscribers
        self.inform_enter_conflict_sub = rospy.Subscriber('inform_enter_conflict', Bool, self.inform_enter_conflict_cb)
        self.cmd_vel_from_img_sub = rospy.Subscriber('cmd_vel_from_img', Twist, self.cmd_vel_from_img_cb)
        self.kinematic_info_sub = rospy.Subscriber('/kinematic_info', KinematicDataArray, self.kinematic_info_cb)
        self.wheel_omega_sub = rospy.Subscriber('wheel_omega', WheelsEncoder, self.wheel_omega_cb)
        self.inter_info_sub = rospy.Subscriber(self.local_inter_info_topic, InterInfoMsg, self.inter_info_cb)
        self.shutdown_sub = rospy.Subscriber("robot_interface_shutdown", Bool, self.signal_shutdown)
        
        # Servers
        self.update_route_server = rospy.Service('update_route_srv', NewRoute, self.update_route_cb)

        # Clients
        self.inter_mng_client = rospy.ServiceProxy('/inter_mng_srv', InterMng)

        self.send_ready_signal()

        self.timer = rospy.Timer(rospy.Duration(1 / 10), self.pub_robot_info)
    
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
        
        # Init
        if new_route == [6, 6, 2]:
            self.curr_route = [6, 6, 2]
            return NewRouteResponse(success=True, message=f"{self.robot_name} start new travel")
        
        # Condition Match             
        if self.curr_route[1] == new_route[0] and self.curr_route[2] == new_route[1]:
            # update local info
            self.curr_route = new_route
            self.local_inter_id = new_route[1]
            self.robot_info.robot_route = self.curr_route
            self.robot_info.robot_p = 0.0
            self.robot_info.robot_enter_time = rospy.get_time()
            self.robot_motion_controller.total_distance_apriltag = 0.0

            # update global info
            self.update_global_inter_info()
            self.update_inter_sub()

            # response
            if new_route[1] == 6 and new_route[2] == 6:
                return NewRouteResponse(success=True, message=f"{self.robot_name} travel end")
            else:
                return NewRouteResponse(success=True, message=f"{self.robot_name} updated new inter info")
        else:
            rospy.logerr(f"Route update sequence mismatch: {self.curr_route} and {new_route}.") 
            return      

    def update_global_inter_info(self):
        """
        Request to update global inter_info.
        """
        rospy.wait_for_service('/inter_mng_srv')
        try:
            req = InterMngRequest()
            req.header.stamp = rospy.Time.now()
            req.robot_id = self.robot_id
            req.last_inter_id = self.curr_route[0]
            req.curr_inter_id = self.curr_route[1]
            resp: InterMngResponse = self.inter_mng_client.call(req)
            if resp.success:
                rospy.loginfo(f'{self.robot_name} - {resp.message}')
            else:
                rospy.logwarn(f'{self.robot_name} - Service response not successful: {resp.message}')
            return
        except rospy.ServiceException as e:
            rospy.logerr(f'{self.robot_name}: Service call failed - {e}')
            return

    def update_inter_sub(self):
        """
        Update local subscribed inter_info/x.
        """

        if hasattr(self, 'inter_info_sub'):
            self.inter_info_sub.unregister()
        
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
        self.local_inter_info.inter_id = inter_info_msg.inter_id
        self.local_inter_info.robot_id_list = inter_info_msg.robot_id_list
        robot_info: List[RobotInfoMsg] = inter_info_msg.robot_info
        self.local_inter_info.robot_info = [
            RobotInfo(
                name=info.robot_name,
                robot_id=info.robot_id,
                robot_route=info.robot_route,
                v=info.robot_v,
                p=info.robot_p,
                coordinate=info.robot_coordinate,
                enter_time=info.robot_enter_time,
                enter_conflict=info.robot_enter_conflict,
                arrive_cp_time=info.robot_arrive_cp_time,
                exit_time=info.robot_exit_time
            ) for info in robot_info
        ]
        return

    def cmd_vel_from_img_cb(self, msg: Twist):
        """
        Receive Twist messages from the vision system.
        """
        cmd_vel = self.make_decision(twist_from_img=msg, inter_info=self.local_inter_info)
        self.cmd_vel_pub.publish(cmd_vel)
        return

    def make_decision(self, twist_from_img: Twist, inter_info: InterInfo) -> Twist:
        """
        Decision-making process based on the robot's current info.
        """
        if self.decision_model.want_to_enter_conflict:
                self.decision_model.enter_permission = self.decision_model.check_enter_permission(self.local_inter_info.robot_info)
                self.robot_info.robot_enter_conflict = self.decision_model.enter_permission
        twist_from_decision = self.decision_model.decision_maker(twist_from_img)
        return twist_from_decision

    def inform_enter_conflict_cb(self, msg: Bool):
        """
        Update the robot's conflict zone entry state based on the received message.
        """
        self.decision_model.want_to_enter_conflict = msg.data
        if not self.decision_model.want_to_enter_conflict:
            self.decision_model.enter_permission = False
            self.robot_info.robot_enter_conflict = False
        return
    
    def kinematic_info_cb(self, kinematic_data_msg: KinematicDataArray):
        """
        Receive self kinematics data and update local info.
        """
        for data in kinematic_data_msg.data:
            if self.robot_id == data.robot_id:
                curr_pose = data.pose
                curr_vel = data.vel

                # update local info
                self.robot_motion_controller.kinematic_recoder(pose=curr_pose, vel=curr_vel)
                self.robot_info.robot_v = curr_vel[0]
                self.robot_info.robot_p = self.robot_motion_controller.total_distance_apriltag
                self.robot_info.robot_coordinate = (curr_pose[1], curr_pose[2])
        return
    
    def wheel_omega_cb(self, wheel_omega_msg: WheelsEncoder):
        """
        Through wheel encoder to calc travel time.
        """
        return self.robot_motion_controller.tick_recorder(msg=wheel_omega_msg)    

    def pub_robot_info(self, event):
        """
        Publisch self robot_info.
        """
        # TODO: update expected robot_arrive_cp_time & robot_exit_time
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