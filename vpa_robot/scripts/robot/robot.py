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

        self.radius = 0.0318
        self.baseline = 0.1

        self.prev_pose_data = []
        self.curr_pose_data = []
        self.pre_position = None
        self.total_travel_distance = 0.0
    
    def kinematic_recoder(self, pose, vel):
        self.prev_pose_data = self.curr_pose_data
        self.curr_pose_data = pose
        self.vel = vel
        return self.calc_total_travel_distance()
        
    def calc_total_travel_distance(self):
        curr_position = (self.curr_pose_data[1], self.curr_pose_data[2])
        
        if self.prev_pose_data is not None:
            pre_position = (self.prev_pose_data[1], self.prev_pose_data[2])
            distance = self.calculate_distance(pre_position, curr_position)
            time = self.curr_pose_data[0] - self.prev_pose_data[0]
            vel = distance / time
            print(vel)
            self.total_travel_distance += distance
            rospy.loginfo(f"Current Travel Distance: {self.total_travel_distance:.3f}")

        return

    @staticmethod
    def calculate_distance(point1, point2):
        threshold = 0.005
        distance = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        if distance <= threshold:
            return 0
        return distance
    
    def vel_caculator(self, msg):
        # 读取左右轮角速度
        omega_left = msg.omega_left
        omega_right = msg.omega_right

        # 计算机器人线速度和角速度
        v = self.radius * (omega_left + omega_right) / 2.0
        omega = self.radius * (omega_right - omega_left) / self.baseline

        rospy.loginfo(f"Linear velocity: {v:.3f} m/s, Angular velocity: {omega:.3f} rad/s")
        return


