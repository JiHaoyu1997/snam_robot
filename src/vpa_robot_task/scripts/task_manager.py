#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

from vpa_robot_task.srv import AssignTask, AssignTaskRequest, AssignTaskResponse

class TaskManager:
    # Properties
    def __init__(self) -> None:
        # Init ROS Node
        rospy.init_node('task_manager')

        # get the robot name, by default it will be the hostname of the robot
        self.robot_name = rospy.get_param('~robot_name', 'db19')

        # Task Lits
        self.task_list = []

        # Intersection Robot List
        self.inter_info = []

        # Publishers
        self.local_brake_pub = rospy.Publisher('local_brake', Bool, queue_size=1)

        # Subscribers


        # Servers
 

        # Clients
        self.assgin_task_client = rospy.ServiceProxy('/assgin_task_srv', AssignTask)

        
        # Init Log
        rospy.loginfo('Task Manager is Online')

        # Start
        self.status_init()

    # Methods
    def status_init(self):
        if self.request_task():
            rospy.loginfo("%s: already confirmed Task List and will release local Brake", self.robot_name)
            msg = Bool()
            msg.data = False
            self.local_brake_pub.publish(msg)
            
    def request_task(self) -> list:
        rospy.loginfo("%s: Waiting for Task Assign Server", self.robot_name)
        rospy.wait_for_service("/assgin_task_srv")
        try:
            resp: AssignTaskResponse = self.assgin_task_client.call(self.robot_name)
            # TODO: task list valid check
            if self.validate_task_list(resp.task_list):
                self.task_list = resp.task_list
                return True
            else:
                pass           
        except rospy.ServiceException as e:
            rospy.logerr(f'{self.robot_name}: Service call fail: {e}')
    
    def update_task(self) -> list:
        pass

    def validate_task_list(self, task_list):
        return True
    
if __name__ == '__main__':
    N = TaskManager()
    rospy.spin()
