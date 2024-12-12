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

import rospy

import math
import threading

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

class PermissionChecker:
    def __init__(self, check_function, callback, check_interval=0.5):
        """
        初始化权限检查器
        
        :param check_function: 用于检查权限的函数，返回 True 或 False
        :param callback: 当权限通过时触发的回调函数
        :param check_interval: 检查权限的时间间隔（秒）
        """
        self._check_function = check_function
        self._callback = callback
        self._check_interval = check_interval
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        """
        启动检查线程
        """
        if self._thread and self._thread.is_alive():
            return  # 如果线程已经运行，不重复启动
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run)
        self._thread.start()

    def stop(self):
        """
        停止检查线程
        """
        if self._thread and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join()

    def _run(self):
        """
        检查权限的循环逻辑
        """
        while not self._stop_event.is_set():
            if self._check_function():
                self._callback()
                break
            self._stop_event.wait(self._check_interval)  # 等待指定间隔