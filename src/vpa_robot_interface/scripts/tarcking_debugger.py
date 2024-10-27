import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import time

class TrackingDebugger:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 存储调试数据
        self.timestamps = []
        self.actual_positions = []
        self.target_positions = []
        self.wheel_velocities = []
        self.control_commands = []
        
        # 测试参数
        self.test_duration = 10.0  # 测试持续时间(秒)
        self.sample_rate = 20  # 采样率(Hz)
        
    def straight_line_test(self):
        """直线行驶测试"""
        start_time = time.time()
        rate = rospy.Rate(self.sample_rate)
        
        while (time.time() - start_time) < self.test_duration:
            # 发送恒定的前进命令
            cmd = Twist()
            cmd.linear.x = 0.3  # 固定线速度
            cmd.angular.z = 0.0  # 期望直线行驶
            self.cmd_pub.publish(cmd)
            
            # 记录控制指令
            self.control_commands.append([cmd.linear.x, cmd.angular.z])
            
            rate.sleep()
            
    def circle_test(self, radius=1.0):
        """圆周运动测试"""
        start_time = time.time()
        rate = rospy.Rate(self.sample_rate)
        
        while (time.time() - start_time) < self.test_duration:
            # 发送圆周运动命令
            cmd = Twist()
            cmd.linear.x = 0.2  # 线速度
            cmd.angular.z = cmd.linear.x / radius  # 角速度
            self.cmd_pub.publish(cmd)
            
            self.control_commands.append([cmd.linear.x, cmd.angular.z])
            rate.sleep()
    
    def odom_callback(self, msg):
        """记录里程计数据"""
        current_time = time.time()
        current_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
        
        self.timestamps.append(current_time)
        self.actual_positions.append(current_pos)
        
    def analyze_data(self):
        """分析收集到的数据"""
        positions = np.array(self.actual_positions)
        
        # 计算路径偏差
        if len(positions) > 1:
            # 计算起点到终点的直线
            start_point = positions[0]
            end_point = positions[-1]
            path_vector = end_point - start_point
            
            # 计算每个点到理想直线的距离
            deviations = []
            for point in positions:
                deviation = np.abs(np.cross(path_vector, point - start_point)) / np.linalg.norm(path_vector)
                deviations.append(deviation)
            
            max_deviation = max(deviations)
            avg_deviation = sum(deviations) / len(deviations)
            
            print(f"最大偏差: {max_deviation:.3f} 米")
            print(f"平均偏差: {avg_deviation:.3f} 米")
            
            # 分析速度稳定性
            cmd_velocities = np.array(self.control_commands)
            linear_velocity_std = np.std([cmd[0] for cmd in self.control_commands])
            angular_velocity_std = np.std([cmd[1] for cmd in self.control_commands])
            
            print(f"线速度标准差: {linear_velocity_std:.3f}")
            print(f"角速度标准差: {angular_velocity_std:.3f}")

def main():
    rospy.init_node('tracking_debugger')
    debugger = TrackingDebugger()
    
    # 执行直线测试
    print("开始直线测试...")
    debugger.straight_line_test()
    
    # 等待数据收集完成
    rospy.sleep(1.0)
    
    # 分析数据
    print("\n分析测试结果:")
    debugger.analyze_data()

if __name__ == '__main__':
    main()