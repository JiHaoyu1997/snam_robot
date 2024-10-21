#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from vpa_robot_interface.msg import WheelsCmd, WheelsEncoder

class DirectYawControl:

    def __init__(self) -> None:
        self.ref_vx   = 0
        self.ref_yz   = 0
        self.v_x_interal = 0
        self.acc_x       = 0
        self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.car_cmd_cb)
        self.sub_imu = rospy.Subscriber("imu",Imu,self.imu_cb)
        self.sub_enc = rospy.Subscriber("wheel_omega", Twist, self.car_cmd_cb)
    
    def car_cmd_cb(self,data:Twist):
        self.ref_vx = data.linear.x
        self.ref_yz = data.angular.z

    def imu_cb(self,data:Imu):
        self.acc_x = 9.8 * data.linear_acceleration.x # m/s^2

