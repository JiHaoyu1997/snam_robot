#!/usr/bin/env python3

import rospy
import socket

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from vpa_robot_interface.msg import WheelsCmd

class JOY2TWIST:
    
    def __init__(self) -> None:
        
        self.robot_name = socket.gethostname()
        self.local_unlock_flag = False
        self.latch_flag        = False
        self.timer = None
        self.sub_joy = rospy.Subscriber('joy',Joy,self.joy_cb)
        self.pub_brk = rospy.Publisher('local_brake',Bool,queue_size=1)
        self.pub_cmd = rospy.Publisher('throttle',WheelsCmd,queue_size=1)
        rospy.loginfo("%s: joystick ready",self.robot_name)
        
    def joy_cb(self,data:Joy):
        axes = data.axes
        buts = data.buttons
        
        LB = buts[4]
        RB = buts[5]
        
        if LB == 1 and RB == 1:
            msg = Bool()
            if not self.latch_flag:
                self.latch_flag = True
                self.timer = rospy.Timer(rospy.Duration(0.5), self.latch_lock, oneshot=True)
                if self.local_unlock_flag:
                    # has been unlock
                    msg.data = True # lock the wheels
                    self.local_unlock_flag = False
                    rospy.loginfo('local brake engaged')
                else:
                    msg.data = False
                    self.local_unlock_flag = True
                self.pub_brk.publish(msg)
        
        left_UD     = axes[1]
        right_LR    = axes[3]
        
        msg = WheelsCmd()
        k = 0.8
        msg.throttle_left   = self.bound_output(left_UD * (1 - k * right_LR))
        msg.throttle_right  = self.bound_output(left_UD * (1 + k * right_LR))
        
        self.pub_cmd.publish(msg)
    
    def bound_output(self,input) -> float:
        if input > 0.8:
            return 0.8
        elif input < -0.3:
            return -0.3
        return input
    
    def latch_lock(self,event):
        self.latch_flag = False
if __name__ == "__main__":
    
    rospy.init_node('joystick_translator') 
    T = JOY2TWIST()
    rospy.spin()       