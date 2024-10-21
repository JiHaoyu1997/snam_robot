#!/usr/bin/env python3

import rospy

class TaskManager:
    # Properties
    def __init__(self) -> None:
        # Init ROS Node
        rospy.init_node('task_manager')

        # get the robot name, by default it will be the hostname of the robot
        self._robot_name     = rospy.get_param('~robot_name', 'db19')

        # Intersection Robot List
        self.inter_info = []

        # Publishers
        
        

        # Subscribers


        # Servers


        # ServiceProxy

        
        # Init Log
        rospy.loginfo('Task Manager is Online')

        # 

    # Methods    
    def _request_task_init(self):
        rospy.loginfo("%s: Waiting for task server", self._robot_name)
        rospy.wait_for_service("/AssignTask")
        self._task_proxy = rospy.ServiceProxy("/AssignTask", AssignTask)
        rospy.loginfo("%s: Task server online", self._robot_name)
    
    def _request_task_service(self,task_index) -> list:
        rospy.loginfo_once('%s: arrive ready line, inquiring task',self._robot_name)
        resp = self._task_proxy(self._robot_name,task_index)
        return resp.node_list
    
if __name__ == '__main__':
    N = TaskManager()
    rospy.spin()
