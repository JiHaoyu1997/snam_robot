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
    def __init__(self, name="", id=0, a=0.0, v=0.0, p=0.0, coordinate=(0.0, 0,0), enter_time=0.0, arrive_cp_time=0.0, exit_time=0.0):
        self.robot_name = name
        self.robot_id = id
        self.robot_a = a  # Acceleration
        self.robot_v = v  # Velocity
        self.robot_p = p  # Position
        self.robot_coordinate = coordinate # Coordinate
        self.robot_enter_time = enter_time
        self.robot_arrive_cp_time = arrive_cp_time
        self.robot_exit_time = exit_time


class InterInfo:
    def __init__(self, inter=0):
        self.inter = inter  # Intersection ID
        self.robot_id_list = []  # List of robot names or IDs
        self.robot_info: List[RobotInfo] = []  # List of RobotInfo instances


class FCFSModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.enter_conflict_zone = False


    def decision_maker(self, twist_from_img: Twist, robot_inter_info: InterInfo):
        if not robot_inter_info.robot_id_list:
            return twist_from_img
        
        print(robot_inter_info.robot_id_list)

        self.pass_permssion = self.robot_id == robot_inter_info.robot_id_list[0]

        if self.enter_conflict_zone and not self.pass_permssion:
            return Twist()  # Stop the robot if it doesn't have permission

        return twist_from_img
    
    
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
        
        #
        self.node_name = 'robot_decision' 

        # Robot name
        self.robot_name = rospy.get_param('~robot_name', 'db19')
        self.robot_id = find_id_by_robot_name(robot_name=self.robot_name)
        self.decision_model = FCFSModel(robot_id=self.robot_id)
        self.robot_motion_controller = RobotMotion(name=self.robot_name, id=self.robot_id) 

        # Initialize route and intersection information
        self.robot_info = RobotInfo(name=self.robot_name)
        self.curr_route = [0, 0, 0]
        self.local_inter_id = 0
        self.local_inter_info = InterInfo()
        self.local_inter_info_topic = f'inter_info/{self.local_inter_id}'

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Subscribers
        self.inform_enter_conflict_sub = rospy.Subscriber('inform_enter_conflict', Bool, self.inform_enter_conflict_cb)
        self.cmd_vel_from_img_sub = rospy.Subscriber('cmd_vel_from_img', Twist, self.cmd_vel_from_img_cb)
        self.kinematic_info_sub = rospy.Subscriber('/kinematic_info', KinematicDataArray, self.kinematic_info_cb)
        self.wheel_omega_sub = rospy.Subscriber('wheel_omega', WheelsEncoder, self.wheel_omega_cb)
        self.inter_info_sub = rospy.Subscriber(self.local_inter_info_topic, InterInfoMsg, self.inter_info_cb)

        # Servers
        self.update_route_server = rospy.Service('update_route_srv', NewRoute, self.update_route_cb)

        # Clients
        self.inter_mng_client = rospy.ServiceProxy('/inter_mng_srv', InterMng)

        self.send_ready_signal()
    
    # Methods
    def send_ready_signal(self):
        rospy.wait_for_service('ready_signal')
        try:
            ready_signal_client = rospy.ServiceProxy('ready_signal', ReadySignal)
            response: ReadySignalResponse = ready_signal_client.call(self.node_name)
            if response.success:
                rospy.loginfo(f"{self.robot_name}: Decision Maker is Online")
                self.curr_route = [6, 6, 2]   
                self.local_inter_id = 6
        except rospy.ServiceException as e:
            rospy.logerr(f"service call failed: {e}")

    def update_route_cb(self, req: NewRouteRequest):
        """Callback for updating route if conditions are met."""
        new_route = [route for route in req.new_route]

        if len(new_route) != 3:
            rospy.logerr("Route message does not contain 3 elements.")
            return NewRouteResponse(success=False, message='Update Error')
             
        if self.curr_route[1] == new_route[0] and self.curr_route[2] == new_route[1]:
            last_inter_id = new_route[0]
            curr_inter_id = new_route[1]
            self.update_global_inter_info(last_inter_id, curr_inter_id)
            self.curr_route = new_route
            self.local_inter_id = curr_inter_id
            self.update_inter_sub()
            return NewRouteResponse(success=True, message=f"{self.robot_name} updated new inter info")
        else:
            rospy.logerr(f"Route update sequence mismatch: {self.curr_route} and {new_route}.")       

    def update_global_inter_info(self, last_inter_id, curr_inter_id):
        """Update global intersection info by calling the service."""
        rospy.wait_for_service('/inter_mng_srv')
        try:
            self.robot_info = RobotInfo(
                name=self.robot_name,
                id=find_id_by_robot_name(robot_name=self.robot_name),
            )

            req = InterMngRequest()
            req.header.stamp = rospy.Time.now()
            req.robot_name = self.robot_name
            req.last_inter_id = last_inter_id
            req.curr_inter_id = curr_inter_id
            req.robot_info = self.robot_info

            resp: InterMngResponse = self.inter_mng_client.call(req)
            if resp.success:
                rospy.loginfo(f'{self.robot_name} - {resp.message}')
            else:
                rospy.logwarn(f'{self.robot_name} - Service response not successful: {resp.message}')
        except rospy.ServiceException as e:
            rospy.logerr(f'{self.robot_name}: Service call failed - {e}')

    def update_inter_sub(self):
        if hasattr(self, 'inter_info_sub'):
            self.inter_info_sub.unregister()

        self.local_inter_info_topic = f'/inter_info/{self.local_inter_id}'
        self.inter_info_sub = rospy.Subscriber(
            self.local_inter_info_topic,
            InterInfoMsg,
            self.inter_info_cb
        )
        rospy.loginfo(f"{self.robot_name} updated inter_sub to {self.local_inter_info_topic}")

    def inter_info_cb(self, inter_info_msg: InterInfoMsg):
        """Updates local intersection information based on the message."""
        self.local_inter_info.inter = inter_info_msg.inter_id
        self.local_inter_info.robot_id_list = inter_info_msg.robot_id_list
        robot_info: List[RobotInfoMsg] = inter_info_msg.robot_info
        self.local_inter_info.robot_info = [
            RobotInfo(
                name=info.robot_name,
                robot_id=info.robot_id,
                a=info.robot_a,
                v=info.robot_v,
                p=info.robot_p,
                coordinate=info.robot_coordinate,
                enter_time=info.robot_enter_time,
                arrive_cp_time=info.robot_arrive_cp_time,
                exit_time=info.robot_exit_time
            ) for info in robot_info
        ]


    def cmd_vel_from_img_cb(self, msg: Twist):
        """
        Receive Twist messages from the vision system.
        """
        cmd_vel = self.make_decision(twist_from_img=msg, robot_inter_info=self.local_inter_info)
        self.cmd_vel_pub.publish(cmd_vel)

    def make_decision(self, twist_from_img: Twist, robot_inter_info: InterInfo) -> Twist:
        """
        Decision-making process based on the robot's current state and received command.
        """
        twist_from_decision = self.decision_model.decision_maker(twist_from_img, robot_inter_info)
        return twist_from_decision

    def kinematic_info_cb(self, kinematic_data_msg: KinematicDataArray):
        for data in kinematic_data_msg.data:
            if self.robot_id == data.robot_id:
                curr_pose = data.pose
                curr_vel = data.vel
                self.robot_motion_controller.kinematic_recoder(pose=curr_pose, vel=curr_vel)
        return
    
    def wheel_omega_cb(self, wheel_omega_msg: WheelsEncoder):
        return self.robot_motion_controller.tick_recorder(msg=wheel_omega_msg)
    
    def inform_enter_conflict_cb(self, msg: Bool):
        if msg:
            self.decision_model.enter_conflict_zone = msg.data


if __name__ == '__main__':
    N = RobotDecision()
    rospy.spin()