#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool, Int8MultiArray
from std_srvs.srv import Trigger, TriggerResponse

from vpa_robot_vision.msg import CrossInfo

from vpa_robot_task.srv import AssignTask, AssignTaskRequest, AssignTaskResponse

class TaskManager:
    # Properties
    def __init__(self) -> None:
        # Init ROS Node
        rospy.init_node('task_manager')

        # get the robot name, by default it will be the hostname of the robot
        self.robot_name = rospy.get_param('~robot_name', 'db19')

        # 
        self.curr_local_brake_status = True

        # Task List
        self.task_list = []

        # Current task index
        self.curr_task_index = 0

        # Current route (two intersections: current and next)
        self.curr_route = []

        # 
        self.update_task_lock = False

        # Publishers
        self.local_brake_pub = rospy.Publisher('local_brake', Bool, queue_size=1)
        self.curr_route_pub = rospy.Publisher('curr_route', Int8MultiArray, queue_size=1)

        # Subscribers
        self.inter_boundary_line_detect_sub = rospy.Subscriber('inter_boundary_line_detect', CrossInfo, self.inter_boundary_line_detect_cb)

        # Servers

        # Clients
        self.assign_task_client = rospy.ServiceProxy('/assgin_task_srv', AssignTask)

        # Init Log
        rospy.loginfo('Task Manager is Online')

        # Start the task manager
        self.status_init()
        
        # Publish local brake status
        self.timer = rospy.Timer(rospy.Duration(1), self.pub_local_brake_status)

        # Publish initial route
        self.timer = rospy.Timer(rospy.Duration(1/10), self.pub_curr_route)

    # Methods
    def inter_boundary_line_detect_cb(self, cross_msg: CrossInfo):
        """ Update Current Route """
        if cross_msg.cross and not self.update_task_lock:
            # AD
            if not (self.curr_route[0] == cross_msg.last_inter_id and self.curr_route[1] == cross_msg.local_inter_id):
                rospy.logwarn(f"Task Update Func Error")
                return
            # BD
            if self.curr_task_index >= len(self.task_list) - 2:
                rospy.logwarn("%s: Task List Index out of bounds!", self.robot_name)
                self.curr_route = [6, 6]
                return

            # Update current route
            self.curr_task_index += 1
            self.curr_route = [self.task_list[self.curr_task_index], self.task_list[self.curr_task_index + 1]]
            rospy.loginfo(f"Updated current route: {self.curr_route}")    

            # 
            self.update_task_lock = True
        
        elif not cross_msg:
            self.update_task_lock = False

    def status_init(self):
        if self.request_task():
            rospy.loginfo("%s: Task List confirmed", self.robot_name)
            
            # Update the 1st route 
            self.curr_route = [self.task_list[self.curr_task_index], self.task_list[self.curr_task_index + 1]]

            # Release local Brake
            self.curr_local_brake_status = False
        else:
            rospy.logwarn("%s: Task Manager initialization failed", self.robot_name)

    def request_task(self) -> bool:
        rospy.loginfo("%s: Waiting for Task Assign Server", self.robot_name)
        rospy.wait_for_service("/assgin_task_srv")
        try:
            resp: AssignTaskResponse = self.assign_task_client(self.robot_name)
            if self.validate_task_list(resp.task_list):
                self.task_list = resp.task_list
                return True
            else:
                rospy.logerr("%s: Task list validation failed", self.robot_name)
                return False          
        except rospy.ServiceException as e:
            rospy.logerr('%s: Service call failed: %s', self.robot_name, e)
            return False

    def validate_task_list(self, task_list) -> bool:
        # TODO: Implement task list validation
        return bool(task_list) and len(task_list) >= 2

    def pub_local_brake_status(self):
        brake_msg = Bool(data=self.curr_local_brake_status)
        self.local_brake_pub.publish(brake_msg)

    def pub_curr_route(self):
        route_msg = Int8MultiArray(data=self.curr_route)
        self.curr_route_pub.publish(route_msg)

if __name__ == '__main__':
    TaskManager()
    rospy.spin()