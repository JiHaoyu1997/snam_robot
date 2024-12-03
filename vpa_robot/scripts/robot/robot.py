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
        self.total_distance_apriltag = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_distance_wheel_omega = 0.0
    
    def kinematic_recoder(self, pose, vel):
        self.prev_pose_data = self.curr_pose_data
        self.curr_pose_data = pose
        self.vel = vel
        return self.calc_total_distance_apriltag()
        
    def calc_total_distance_apriltag(self):
        if len(self.prev_pose_data) <= 3:
            return

        prev_position = (self.prev_pose_data[1], self.prev_pose_data[2])
        curr_position = (self.curr_pose_data[1], self.curr_pose_data[2])
        distance = self.calculate_distance(prev_position, curr_position)        
        self.total_distance_apriltag += distance
        rospy.loginfo(
            f"AprilTag -- Linear vel: {self.vel[0]:.3f} m/s, Ang vel: {self.vel[1]:.3f} rad/s, Tot Dis: {self.total_distance_apriltag:.3f}m")

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
            v = self.radius * (omega_left + omega_right) / 2.0  # 线速度
            omega = self.radius * (omega_right - omega_left) / self.baseline  # 角速度

            dt = 1 / 20
            distance_moved = self.update_position(v, omega, dt)

            # 打印当前速度和位置
            rospy.loginfo(
                f"Wheel Omega -- Linear vel: {v:.3f} m/s, Ang vel: {omega:.3f} rad/s, Tot Dis: {self.total_distance_wheel_omega:.3f} m")


    def update_position(self, v, omega, dt):
            """
            计算路径长度
            """
            if omega == 0:  # 如果角速度为0，机器人沿直线移动
                # 更新位置
                dx = v * dt * math.cos(self.theta)
                dy = v * dt * math.sin(self.theta)
                distance = v * dt  # 距离为线速度 * 时间

            else:  # 如果有角速度，计算弧度变化
                # 计算角度变化
                dtheta = omega * dt
                # 计算弧长（路径长度）
                distance = abs(v / omega * dtheta)  # 路径长度 = 半径 * 角度变化

                # 更新位置：弧线运动
                dx = (v / omega) * (math.sin(self.theta + dtheta) - math.sin(self.theta))
                dy = (v / omega) * (math.cos(self.theta) - math.cos(self.theta + dtheta))

            # 更新机器人位置和朝向
            self.x += dx
            self.y += dy
            self.theta += omega * dt

            # 保证角度在[-pi, pi]范围内
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

            # 累加总路径长度
            self.total_distance_wheel_omega += distance

            return distance