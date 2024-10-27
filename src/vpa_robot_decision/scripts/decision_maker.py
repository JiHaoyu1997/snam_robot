#!/usr/bin/env python3

import rospy
from typing import List

from vpa_robot.robot.robot import find_id_by_robot_name

from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist

from vpa_robot_decision.msg import RobotInfo as RobotInfoMsg
from vpa_robot_decision.msg import InterInfo as InterInfoMsg

from vpa_robot_decision.srv import InterMng, InterMngRequest, InterMngResponse

class RobotInfo:
    def __init__(self, name="", id=0, a=0.0, v=0.0, p=0.0, enter_time=0.0, arrive_cp_time=0.0, exit_time=0.0):
        self.robot_name = name
        self.robot_id = id
        self.robot_a = a  # Acceleration
        self.robot_v = v  # Velocity
        self.robot_p = p  # Position
        self.robot_enter_time = enter_time
        self.robot_arrive_cp_time = arrive_cp_time
        self.robot_exit_time = exit_time

class InterInfo:
    def __init__(self, inter=0):
        self.inter = inter  # Intersection ID
        self.robot_id_list = []  # List of robot names or IDs
        self.robot_info = []  # List of RobotInfo instances

class DecisionMaker:
    def __init__(self) -> None:
        # Initialize ROS node
        rospy.init_node('decision_maker')

        # Robot name
        self.robot_name = rospy.get_param('~robot_name', 'db19')

        # Initialize route and intersection information
        self.robot_info = RobotInfo(name=self.robot_name)
        self.curr_route = [0, 0]
        self.local_inter_id = 0
        self.local_inter_info = InterInfo()
        self.local_inter_info_topic = f'inter_info/{self.local_inter_id}'

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Subscribers
        self.curr_route_sub = rospy.Subscriber('curr_route', Int8MultiArray, self.curr_route_cb)        
        self.inter_info_sub = rospy.Subscriber(self.local_inter_info_topic, InterInfoMsg, self.inter_info_cb)
        self.cmd_vel_from_img_sub = rospy.Subscriber('cmd_vel_from_img', Twist, self.cmd_vel_from_img_cb)

        # Service client
        self.inter_mng_client = rospy.ServiceProxy('/inter_mng_srv', InterMng)

        rospy.loginfo('Decision Maker is Online')

    def curr_route_cb(self, route_msg: Int8MultiArray):
        """Callback for the current route, updates route if conditions are met."""
        if len(route_msg.data) != 2:
            rospy.logerr("Route message does not contain two elements.")
            return
        
        new_route = [route_msg.data[0], route_msg.data[1]]

        # 
        if self.curr_route == [0, 0]:
            self.curr_route = new_route
            return

        # 
        if self.curr_route == new_route:
            # rospy.loginfo("Current route unchanged.")
            pass
        elif self.curr_route[1] == new_route[0]:  # Valid route update
            last_inter_id, curr_inter_id = self.curr_route
            self.update_global_inter_info(last_inter_id, curr_inter_id)
            self.curr_route = new_route
            self.local_inter_id = curr_inter_id
            self.update_inter_sub()
        else:
            rospy.logerr("Route update sequence mismatch.")

    def update_inter_sub(self):
        if hasattr(self, 'inter_info_sub'):
            self.inter_info_sub.unregister()

        self.local_inter_info_topic = f'inter_info/{self.local_inter_id}'
        self.inter_info_sub = rospy.Subscriber(
            self.local_inter_info_topic,
            InterInfoMsg,
            self.inter_info_cb
        )
        rospy.loginfo(f"Updated Intersection Subscription to {self.local_inter_info_topic}")

    def update_global_inter_info(self, last_inter_id, curr_inter_id):
        """Update global intersection info by calling the service."""
        rospy.wait_for_service('/robot_inter_mng_srv')
        try:
            self.robot_info = RobotInfo(
                name=self.robot_name,
                id=4,
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
                enter_time=info.robot_enter_time,
                arrive_cp_time=info.robot_arrive_cp_time,
                exit_time=info.robot_exit_time
            ) for info in robot_info
        ]

    def cmd_vel_from_img_cb(self, msg: Twist):
        """Publishes Twist messages received from the vision system."""
        # Make a decision or directly forward based on image input
        cmd_vel = self.make_decision(twist_from_img=msg, robot_inter_info=self.local_inter_info)
        self.cmd_vel_pub.publish(cmd_vel)

    def make_decision(self, twist_from_img: Twist, robot_inter_info: InterInfo) -> Twist:
        """Decision-making process based on the robot's current state and received command."""
        # Example: Placeholder - logic to be implemented
        rospy.logdebug(f"Making decision based on intersection {robot_inter_info.inter} and incoming command.")
        return twist_from_img

if __name__ == '__main__':
    N = DecisionMaker()
    rospy.spin()
