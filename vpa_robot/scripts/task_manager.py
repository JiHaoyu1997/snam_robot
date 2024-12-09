#!/usr/bin/env python3

import rospy

import time

from robot.robot import robot_dict

from std_msgs.msg import Bool, Int8MultiArray

from vpa_robot.srv import AssignRoute, AssignRouteRequest, AssignRouteResponse
from vpa_robot.srv import AssignTask, AssignTaskRequest, AssignTaskResponse
from vpa_robot.srv import NewTaskList, NewTaskListRequest, NewTaskListResponse
from vpa_robot.srv import ReadySignal, ReadySignalRequest, ReadySignalResponse

class TaskManager:
    # Properties
    def __init__(self) -> None:
        # Init ROS Node
        rospy.init_node('task_manager')

        # get the robot name, by default it will be the hostname of the robot
        self.robot_name = rospy.get_param('~robot_name', 'db19')

        # 
        self.node_status = {
            'robot_vision': False,
            'robot_decision': False,
        }

        # Task List
        self.task_list = []

        # Current task index
        self.curr_task_index = 0

        # Current route (three intersections: last, current and next)
        self.curr_route = [6, 6, 2]

        # Publishers
        self.curr_route_pub = rospy.Publisher('curr_route', Int8MultiArray, queue_size=1)

        # Subscribers

        # Servers
        self.node_ready_handle_server = rospy.Service('ready_signal', ReadySignal, self.ready_signal_cb)
        self.assign_route_server = rospy.Service('assign_route_srv', AssignRoute, self.assign_route_cb)
        self.new_task_list_server = rospy.ServiceProxy('new_task_list_srv', NewTaskList, self.req_new_task_list_cb)

        # Clients
        self.assign_task_client = rospy.ServiceProxy('/assign_task_srv', AssignTask)

        # Start the task manager
        self.status_init()
    
        # Publish initial route
        self.timer = rospy.Timer(rospy.Duration(1 / 10), self.pub_curr_route)

    # Methods
    def status_init(self):
        while not all(self.node_status.values()):
            rospy.loginfo("Waiting for all nodes to ready")
            time.sleep(1)
            
        rospy.loginfo("All nodes are ready! Task Manager is Online")

        if self.request_task():
            rospy.loginfo(f"{self.robot_name}: Task List confirmed: {self.task_list}")
            
        else:
            rospy.logwarn("%s: Task Manager initialization failed", self.robot_name)

    def request_task(self) -> bool:
        rospy.loginfo("%s: Waiting for Task Assign Server", self.robot_name)
        rospy.wait_for_service("/assign_task_srv")

        try:
            req = AssignTaskRequest(robot_name = self.robot_name)
            resp: AssignTaskResponse = self.assign_task_client.call(req)

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

    def ready_signal_cb(self, req: ReadySignalRequest):
        NODE_NAME = req.node_name 

        if NODE_NAME in self.node_status:
            self.node_status[NODE_NAME] = True    
            return ReadySignalResponse(success=True)
        
        else:
            rospy.logwarn("Node not recognized")
            return ReadySignalResponse(success=False)
    
    def assign_route_cb(self, req: AssignRouteRequest):
        """ 
        Assign Current Route 
        """

        last_inter_id = req.last_inter_id
        next_inter_id = req.next_inter_id
        
        # Abnormal Detection
        if not (self.curr_route[1] == last_inter_id and self.curr_route[2] == next_inter_id):
            rospy.logerr(f"Route Update Error")
            self.curr_route = [0, 0 ,0]
            pub = rospy.Publisher('robot_interface_shutdown', Bool, queue_size=1)
            msg = Bool()
            msg.data = True
            pub.publish(msg)
            rospy.signal_shutdown('Abnormal is detected, shutting down the node.')

        # Boundary Detection
        if self.curr_task_index >= len(self.task_list) - 2:
            rospy.logwarn("%s: Task List Index out of bounds!", self.robot_name)
            self.curr_task_index = 0
            self.curr_route = [2, 6, 6]
            return AssignRouteResponse(route=self.curr_route)
        
        # Update current route
        self.curr_task_index += 1      
        self.curr_route = [
            self.task_list[self.curr_task_index - 1], 
            self.task_list[self.curr_task_index], 
            self.task_list[self.curr_task_index + 1]
        ]
        rospy.loginfo(f"{self.robot_name} updated current route: {self.curr_route}")   
        
        return AssignRouteResponse(route=self.curr_route)
    
    def req_new_task_list_cb(self, req: NewTaskListRequest):
        robot_id = req.robot_id
        robot_name =robot_dict[robot_id]
        try:
            resp: AssignTaskResponse = self.assign_task_client.call(robot_name)
            self.task_list = resp.task_list
            return NewTaskListResponse(success=True, message=f"{self.robot_name} req new task list successfully: {self.task_list}")
        except rospy.ServiceException as e:
            rospy.logerr('%s: Service call failed: %s', self.robot_name, e)
            return NewTaskListResponse(success=False, message=f"{self.robot_name} req new task list failed")
        
    def pub_curr_route(self, event):
        route_msg = Int8MultiArray(data=self.curr_route)
        self.curr_route_pub.publish(route_msg)


if __name__ == '__main__':
    N = TaskManager()
    rospy.spin()