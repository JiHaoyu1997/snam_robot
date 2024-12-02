robot_dict = {
    0:'origin',
    1:'mingna',
    2:'vivian',
    3:'gina',
    4:'lucas',
    5:'daisy',
    6:'henry',
    7:'dorie',
    8:'luna',
    9:'robert',
    10:'fiona',
    11:'bingda',
    12:'jetson1'
}


def find_id_by_robot_name(robot_name):
    return next((key for key, value in robot_dict.items() if value == robot_name), None)

import rospy

import math

class RobotMotion:

    def __init__(self, name="",id=0):
        self.robot_name = name
        self.robot_id = id
        self.curr_pose_data = []
        self.pre_position = None
        self.total_travel_distance = 0.0
    
    def kinematic_recoder(self, pose, vel):
        self.curr_pose_data = pose
        self.vel = vel
        return self.calc_total_travel_distance()
        
    def calc_total_travel_distance(self):
        curr_position = (self.curr_pose_data[1], self.curr_pose_data[2])

        if self.pre_position is not None:
            distance = self.calculate_distance(self.pre_position, curr_position)
            self.total_travel_distance += distance
            # rospy.loginfo(f"Current Travel Distance: {self.total_travel_distance: 5f}")

        self.pre_position = curr_position
        return

    @staticmethod
    def calculate_distance(point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)