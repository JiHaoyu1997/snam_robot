import math
import threading
from map.map import find_lane_total_distance
from vpa_robot.msg import RobotInfo as RobotInfoMsg

robot_dict = {
    1:'mingna',
    2:'vivian',
    6:'henry',
    7:'dorie',
    8:'luna',
    9:'robert',
    10:'fiona',
}

def find_id_by_robot_name(robot_name):
    return next((key for key, value in robot_dict.items() if value == robot_name), None)


class RobotInfo:
    def __init__(
            self, 
            name="", 
            robot_id=0, 
            robot_route=[0, 0, 0], 
            v=0.0, 
            p=0.0, 
            coordinate=(0.0, 0.0), 
            enter_conflict=False,
            robot_enter_lane_time = 0.0,
            robot_estimated_arrive_conflict_time = 0.0,
            robot_arrival_conflict_time = 0.0,
            robot_enter_conflict_time = 0.0,
            robot_arrive_cp_time = 0.0,
            robot_exit_time = 0.0,
                 ):
        self.robot_name = name
        self.robot_id = robot_id
        self.robot_route = robot_route
        self.robot_v = v
        self.robot_p = p
        self.robot_coordinate = coordinate
        self.robot_enter_conflict = enter_conflict

        # time info
        self.robot_enter_lane_time = robot_enter_lane_time
        self.robot_estimated_arrive_conflict_time = robot_estimated_arrive_conflict_time   
        self.robot_arrival_conflict_time = robot_arrival_conflict_time
        self.robot_enter_conflict_time = robot_enter_conflict_time
        self.robot_arrive_cp_time = robot_arrive_cp_time
        self.robot_exit_time = robot_exit_time


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
        msg.robot_enter_conflict

        # time info
        msg.robot_enter_lane_time = self.robot_enter_lane_time
        msg.robot_estimated_arrive_conflict_time = self.robot_estimated_arrive_conflict_time
        msg.robot_arrival_conflict_time = self.robot_arrival_conflict_time
        msg.robot_enter_conflict_time = self.robot_enter_conflict_time
        msg.robot_arrive_cp_time = self.robot_arrive_cp_time
        msg.robot_exit_time = self.robot_exit_time
        return msg
    
    def calc_lane_travel_time(self):
        lane_time =  self.robot_arrival_conflict_time - self.robot_enter_lane_time
        return lane_time

    def calc_wait_time(self):
        wait_time = self.robot_enter_conflict_time - self.robot_arrival_conflict_time
        return wait_time
    
    def calc_conflict_zone_travel_time(self):
        cz_time = self.robot_exit_time - self.robot_enter_conflict_time
        return cz_time

class RobotMotion:

    def __init__(self, name="",robot_id=0):
        self.robot_name = name
        self.robot_id = robot_id

        self.radius = 0.0318
        self.baseline = 0.1
        self.ticks_per_circle = 135

        self.prev_pose_data = []
        self.curr_pose_data = []
        self.total_distance_apriltag = 0.0
        self.vel_history_apriltag = []

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_distance_wheel_omega = 0.0
        self.vel_history_wheel_omega = []

    
    def kinematic_recoder(self, pose, vel):
        self.prev_pose_data = self.curr_pose_data
        self.curr_pose_data = pose
        self.vel = vel
        self.vel_history_apriltag.append(vel)
        return self.calc_total_distance_apriltag()
        
    def calc_total_distance_apriltag(self):
        if len(self.prev_pose_data) <= 3:
            return

        prev_position = (self.prev_pose_data[1], self.prev_pose_data[2])
        curr_position = (self.curr_pose_data[1], self.curr_pose_data[2])
        distance = self.calculate_distance(prev_position, curr_position)        
        self.total_distance_apriltag += distance
        # rospy.loginfo(f"AprilTag -- Linear vel: {self.vel[0]:.3f} m/s, Ang vel: {self.vel[1]:.3f} rad/s, Tot Dis: {self.total_distance_apriltag:.3f}m")

        return

    @staticmethod
    def calculate_distance(point1, point2):
        threshold = 0.02
        distance = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        if distance <= threshold:
            return 0
        return distance
    
    def tick_recorder(self, msg):
            # 读取左右轮角速度
            left_ticks = msg.left_ticks
            right_ticks = msg.right_ticks

            s_left = left_ticks / self.ticks_per_circle * 2 * math.pi * self.radius
            s_right = right_ticks / self.ticks_per_circle * 2 * math.pi * self.radius

            # 打印当前速度和位置
            # rospy.loginfo(f"Wheel Omega -- Left Wheel Distance: {s_left:.3f} m, Right Wheel Distance: {s_right:.3f}m.")

    def calc_estimated_arrive_conflict_time(self, route):
        curr_lane_total_distance = find_lane_total_distance(last=route[0], current=route[1], next=route[2])
        distance = curr_lane_total_distance - self.total_distance_apriltag
        vel = self.vel
        if vel == 0.0:
            return 0.0
        else:
            est_time = distance / vel[0]
            return est_time