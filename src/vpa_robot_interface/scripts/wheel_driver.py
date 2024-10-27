#!/usr/bin/python3

import os
import time
import rospy
import socket
import threading

from math import fabs, floor

# msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# server
from dynamic_reconfigure.server import Server

from vpa_robot_interface.msg import WheelsCmd, WheelsEncoder, LeftWheelT
from vpa_robot_interface.cfg import omegaConfig

from pid_controller.pi_format import PIController
from dt_config.dt_hardware_settings import MotorDirection, HATv2

class WheelDriver:

    LEFT_MOTOR_MIN_PWM  = 60        #: Minimum speed for left motor
    LEFT_MOTOR_MAX_PWM  = 255       #: Maximum speed for left motor

    RIGHT_MOTOR_MIN_PWM = 60        #: Minimum speed for right motor
    RIGHT_MOTOR_MAX_PWM = 255       #: Maximum speed for right motor
    
    SPEED_TOLERANCE     = 1.0e-2    #: Speed tolerance level

    def __init__(self) -> None:
        self.hat        = HATv2()
        self.leftMotor  = self.hat.get_motor(1, "left")
        self.rightMotor = self.hat.get_motor(2, "right")
        self.leftThrottle   = 0.0
        self.rightThrottle  = 0.0
        self._pwm_update()

    def set_wheels_throttle(self, left: float, right: float):
        """
        Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1
           is_test_cmd (:obj:`bool`): whether this is a command issue by the hardware test
        """

        self.leftThrottle  = left
        self.rightThrottle = right
        self._pwm_update()

    def _pwm_update(self):
        """
        Sends commands to the microcontroller.

        Updates the current PWM signals (left and right) according to 
        the linear velocities of the motors. The requested speed gets
        thresholded.
        """

        vl = self.leftThrottle
        vr = self.rightThrottle

        pwml = self._pwm_value(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self._pwm_value(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)

        leftMotorMode   = 0
        rightMotorMode  = 0

        if fabs(vl) < self.SPEED_TOLERANCE:
            pwml = 0
        elif vl > 0:
            leftMotorMode = MotorDirection.FORWARD
        elif vl < 0:
            leftMotorMode = MotorDirection.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            pwmr = 0
        elif vr > 0:
            rightMotorMode = MotorDirection.FORWARD
        elif vr < 0:
            rightMotorMode = MotorDirection.BACKWARD

        # print(f"Left Throttle IS {self.leftThrottle} | PWML IS {pwml}")
        
        self.leftMotor.set(leftMotorMode, pwml)
        self.rightMotor.set(rightMotorMode, pwmr)
    
    def _pwm_value(self, v, min_pwm, max_pwm):
        """
        Transforms the requested speed into an int8 number.

        Args:
            v (:obj:`float`): requested speed, should be between -1 and 1.
            min_pwm (:obj:`int8`): minimum speed as int8
            max_pwm (:obj:`int8`): maximum speed as int8
        """

        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (max_pwm - min_pwm) + min_pwm))
        return min(pwm, max_pwm)

    def __del__(self):
        """
        Destructor method.

        Releases the motors and deletes tho object.
        """

        self.leftMotor.set(MotorDirection.RELEASE)
        self.rightMotor.set(MotorDirection.RELEASE)
        del self.hat
    
class WheelDriverNode:

    def __init__(self) -> None:

        rospy.on_shutdown(self.shut_hook)
        
        # Get the vehicle name
            # self.veh_name         = rospy.get_namespace().strip("/")
            # if len(self.veh_name) == 0:
            #     self.veh_name = 'db19'
        self.veh_name       = socket.gethostname()

        # Driver
        self.direct_mode    = rospy.get_param('~direct_mode', False)
        self.driver         = WheelDriver()

        # PI Controller
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(script_dir,'adafruit_drivers/kinematics.py')
        if os.path.exists(filepath):
            pass
            # rospy.loginfo("%s: load customize tuning",self.veh_name)
            # from adafruit_drivers.kinematics import kp,ki
            # self.kp     = kp
            # self.ki     = ki
        else:
            self.kp     = 0.1
            self.ki     = 0.005

        self.omega_controller_left  = PIController(ki=self.ki, kp=self.kp)
        self.omega_controller_right = PIController(ki=self.ki, kp=self.kp)

        self.omega_left_ref     = 0
        self.omega_right_ref    = 0

        self.omega_left_sig     = 0
        self.omega_right_sig    = 0

        self.throttle_left      = 0
        self.throttle_right     = 0

        # Kinematics
        self._v_max     = 1         # max longitudinal speed m/s
        self._omega_max = 8         # max yaw rate rad/s
        self._baseline  = 0.1       # gap between wheels m
        self._radius    = 0.0318    # radius of wheels

        # Trim
        if os.path.exists(filepath):
            pass
            # from adafruit_drivers.kinematics import trim
            # self.trim = trim
        else:
            self.trim = 0

        # Global and Local Brake Status
        self.estop = True
        rospy.loginfo("%s: global brake activated", self.veh_name)
        self.local_estop = True
        rospy.loginfo("%s: local brake activated", self.veh_name)

        # Subscribers
            # self.sub_cmd     = rospy.Subscriber("wheels_cmd", WheelsCmd, self.wheels_cmd_cb, queue_size=1)
        if not self.direct_mode:
            self.sub_car_cmd    = rospy.Subscriber("cmd_vel", Twist, self.car_cmd_cb, queue_size=1)
            self.sub_wheel_enc  = rospy.Subscriber("wheel_omega", WheelsEncoder, self.wheel_omega_cb, queue_size=1)
            # self.sub_change_left_wheel_t = rospy.Subscriber("change_left_wheel_t", LeftWheelT, self.change_left_wheel_t_cb, queue_size=1)
        else:
            self.sub_wheel_cmd  = rospy.Subscriber("throttle", WheelsCmd,self.wheel_direct_cb, queue_size=1)

        self.sub_e_stop         = rospy.Subscriber("/global_brake", Bool, self.estop_cb, queue_size=1)
        self.sub_local_e_stop   = rospy.Subscriber("local_brake", Bool, self.estop_local_cb, queue_size=1)
        
        self.sub_node_shutdown  = rospy.Subscriber("robot_interface_shutdown", Bool, self.signal_shut)
        
        # Publishers
        self.pub_wheel_debug    = rospy.Publisher('wheel_ref', WheelsCmd, queue_size=1)
            # self.pub_wheel_dir = rospy.Publisher('wheel_direction')
        
        # Servers
        self.srv = Server(omegaConfig, self.dynamic_reconfigure_callback)
        rospy.loginfo("%s: wheel drivers ready",self.veh_name)

    def car_cmd_cb(self, msg_car_cmd: Twist) -> None:

        if self.estop:
            self.omega_right_ref    = 0
            self.omega_left_ref     = 0

        else:
            msg_car_cmd.linear.x    = max(min(msg_car_cmd.linear.x, self._v_max), -self._v_max)
            msg_car_cmd.angular.z   = max(min(msg_car_cmd.angular.z, self._omega_max), -self._omega_max)

            self.omega_right_ref    = ((msg_car_cmd.linear.x + 0.5 * msg_car_cmd.angular.z * self._baseline) / self._radius) * (1 + self.trim)
            self.omega_left_ref     = ((msg_car_cmd.linear.x - 0.5 * msg_car_cmd.angular.z * self._baseline) / self._radius) * (1 - self.trim)
            
        # print('ref', self.omega_left_ref, self.omega_right_ref)

        msg_wheel_cmd = WheelsCmd()
        msg_wheel_cmd.vel_left  = self.omega_left_ref
        msg_wheel_cmd.vel_right = self.omega_right_ref        
        self.pub_wheel_debug.publish(msg_wheel_cmd)

    def wheel_omega_cb(self, msg: WheelsEncoder) -> None:

        if self.estop or self.local_estop:
            self.driver.set_wheels_throttle(left=0, right=0)
            self.omega_controller_left.reset_controller()
            self.omega_controller_right.reset_controller()
        else:
            self.omega_left_sig     = msg.omega_left
            self.omega_right_sig    = msg.omega_right

            # print('signal', self.omega_left_sig)

            self.throttle_left      = self.omega_controller_left.pi_control(self.omega_left_ref, self.omega_left_sig)
            self.throttle_right     = self.omega_controller_right.pi_control(self.omega_right_ref, self.omega_right_sig)
            
            if self.throttle_left > 1:
                self.throttle_left = 1
            elif self.throttle_left < -0.5:
                self.throttle_left = -0.5

            if self.throttle_right > 1:
                self.throttle_right = 1
            elif self.throttle_right < -0.5:
                self.throttle_right = -0.5
            
            if self.omega_left_ref == 0:
                self.throttle_left = 0
                self.omega_controller_left.reset_controller()

            if self.omega_right_ref == 0:
                self.throttle_right = 0
                self.omega_controller_right.reset_controller()     

            # print('throttle',self.throttle_left, self.throttle_right)
            # self.throttle_left=1.1e-2
            # self.throttle_right=1.1e-2
            self.driver.set_wheels_throttle(left=self.throttle_left, right=self.throttle_right)
    
    def change_left_wheel_t_cb(self, msg: LeftWheelT):
        self.throttle_left = msg.throttle_left
        rospy.loginfo_once('%s: left wheel throttle: %s',self.veh_name, str(msg.throttle_left))

    def estop_cb(self, msg: Bool) -> None:
        self.estop = msg.data
        rospy.loginfo_once('%s: global brake: %s',self.veh_name,str(msg.data))

    def estop_local_cb(self, msg: Bool) -> None:
        self.local_estop = msg.data
        rospy.loginfo_once('%s: local brake: %s',self.veh_name,str(msg.data))        
    
    def signal_shut(self, msg:Bool):
        if msg.data:
            rospy.signal_shutdown('wheel driver node shutdown')

    def shut_hook(self) -> None:
        self.estop = True
        self.driver.set_wheels_throttle(left=0,right=0)
        self.driver = None
        rospy.loginfo("%s: Wheel driver shutdown", self.veh_name)

    def wheel_direct_cb(self,msg:WheelsCmd) -> None:
        
        self.throttle_left  = msg.throttle_left
        self.throttle_right = msg.throttle_right
        
        if not self.estop and not self.local_estop:
            self.driver.set_wheels_throttle(left=self.throttle_left,right=self.throttle_right)
        else:
            self.driver.set_wheels_throttle(left=0,right=0)
            self.omega_controller_left.reset_controller()
            self.omega_controller_right.reset_controller()            

    def dynamic_reconfigure_callback(self, config, level):
        self.kp = config.kp
        self.ki = config.ki
        self.omega_controller_left.update_controller_param(self.kp, self.ki)
        self.omega_controller_right.update_controller_param(self.kp, self.ki)
        return config
    
if __name__ == '__main__':
    try:
        rospy.init_node("wheel_driver")
        N = WheelDriverNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard Shutdown')