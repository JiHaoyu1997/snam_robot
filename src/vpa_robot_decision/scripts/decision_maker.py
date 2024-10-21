#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool

from vpa_trafficmanager.msg import RobotInterInfo
from vpa_trafficmanager.srv import RobotInterMng, RobotInterMngRequest, RobotInterMngResponse

class DecisionMaker:
    # Properties
    def __init__(self) -> None:
        # Init ROS Node
        rospy.init_node('decision_maker')

        # Intersection Info Init
        self.robot_name = rospy.get_param('~robot_name', 'db19')
        self.task_list = []
        self.inter_local_index = 0
        self.robot_inter_info_local = []

        self.enter_line_true_count = 0
        self.entered = False
        # Publishers
        

        # Subscribers
        self.enter_inter_sub = rospy.Subscriber('enter_line_detect', Bool, self.enter_inter_cb) 
        self.robot_inter_info_sub = rospy.Subscriber('/robot_inter_info', RobotInterInfo, self.robot_inter_info_sub_cb)

        # Servers
        self.robot_inter_mng_service = rospy.ServiceProxy('/robot_inter_mng_srv', RobotInterMng)

        # ServiceProxy


        rospy.loginfo('Decision Maker is Online')
    
    # Methods
    def enter_inter_cb(self, msg: Bool):
        # 
        if msg.data:
            self.enter_line_true_count += 1

            if self.enter_line_true_count >= 3 and not self.entered:

                rospy.loginfo(f'{self.robot_name} is going to req a regist')
                from_inter_index = 2
                to_inter_index = 5
                # from_inter_index = self.task_list[0]
                # to_inter_index = self.task_list[1]
                self.update_location(from_inter_index, to_inter_index)

                self.entered = True

        else:
            self.enter_line_true_count = 0
            self.entered = False

    def update_location(self, from_inter_index, to_inter_index):   
        # 
        rospy.wait_for_service('/robot_inter_mng_srv')
        
        try:
            # 
            req = RobotInterMngRequest()
            req.header.stamp = rospy.Time.now()
            req.robot_name = self.robot_name
            req.from_inter_index = from_inter_index
            req.to_inter_index = to_inter_index
            
            # 
            resp: RobotInterMngResponse = self.robot_inter_mng_service.call(req)
            
            if resp.success:
                rospy.loginfo(f'{self.robot_name} {resp.message}')
                self.inter_local_index = to_inter_index
            else:
                pass
        except rospy.ServiceException as e:
            rospy.logerr(f'{self.robot_name}: Service call fail: {e}')    
    
    def robot_inter_info_sub_cb(self, msg: RobotInterInfo):
        inter_local = f'inter_{self.inter_local_index}'
        self.robot_inter_info_local = getattr(msg, inter_local, [])

        if isinstance(self.robot_inter_info_local, tuple):
            self.robot_inter_info_local = list(self.robot_inter_info_local)
            rospy.logwarn('tuple')
        
        if self.robot_inter_info_local is not None:
            rospy.loginfo(self.robot_inter_info_local)
        else:
            rospy.logwarn(f"{self.robot_name} not found in RobotInterInfo message.")

    
if __name__ == '__main__':
    N = DecisionMaker()
    rospy.spin()