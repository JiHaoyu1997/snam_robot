#!/usr/bin/env python3

import rospy
from enum import IntEnum

import RPi.GPIO as GPIO
import socket
from vpa_robot_interface.msg import WheelsCmd,WheelsEncoder
from math import pi
from std_msgs.msg import Bool
class WheelDirection(IntEnum):
    FORWARD = 1
    REVERSE = -1

class WheelEncoderDriver:

    def __init__(self, gpio_pin, callback):

        if not 1 <= gpio_pin <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")
        # validate callback
        if not callable(callback):
            raise ValueError("The callback object must be a callable object")
        # configure GPIO pin
        self._gpio_pin = gpio_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(gpio_pin, GPIO.IN)
        GPIO.add_event_detect(gpio_pin, GPIO.RISING, callback=self._cb)

        self._callback = callback

        self._ticks = 0
        
        self._direction = 1 # because the encodes has one single phase, no reliable to get direction now


        
    def get_direction(self) -> WheelDirection:
        return self._direction

    def set_direction(self, direction: WheelDirection):
        self._direction = direction

    def _cb(self, _):
        self._ticks += self._direction
        self._callback(self._ticks)

    def __del__(self):
        GPIO.remove_event_detect(self._gpio_pin)

class WheelEncodersNode:

    def __init__(self) -> None:
        self.veh_name           = socket.gethostname()
        
        self.seq = 0
        self._publish_frequency = 20
        self._resolution        = 135
        self.left_gpio          = 18
        self.right_gpio         = 19

        self._tick_left         = 0
        self._tick_right        = 0

        self._tick_left_last    = 0
        self._tick_right_last   = 0

        self._last_tick_timing_left     = 0
        self._last_tick_timing_right    = 0

        self.omega_left     = 0
        self.omega_right    = 0
        
        self.omega_window_left      = []
        self.omega_window_right     = []
        self.window_length          = 10
        self.window_pointer_left    = 0
        self.window_pointer_right   = 0

        self.sub_dir       = rospy.Subscriber('wheel_ref',WheelsCmd,self.dir_cb)
        self.pub_omega      = rospy.Publisher('wheel_omega',WheelsEncoder,queue_size=1)

        self.left_driver    = WheelEncoderDriver(self.left_gpio,self.left_enc_cb)
        self.right_driver   = WheelEncoderDriver(self.right_gpio,self.right_enc_cb) 

        self._timer       = rospy.Timer(rospy.Duration(1.0 / self._publish_frequency), self._cb_publish)
        self._timer_omega = rospy.Timer(rospy.Duration(1/20),self._omega_reduce_cb)

        rospy.loginfo("%s: wheel encoders ready",self.veh_name)
        rospy.Subscriber("robot_interface_shutdown", Bool, self.signal_shut)

    def signal_shut(self,msg:Bool):
        if msg.data:
            rospy.signal_shutdown('encoder node shutdown')

    def dir_cb(self,msg:WheelsCmd):
        
        if msg.throttle_left < 0:
            self.left_driver.set_direction(-1)
        else:
            self.left_driver.set_direction(1)
            
        if msg.throttle_right < 0:
            self.right_driver.set_direction(-1)
        else:
            self.right_driver.set_direction(1)
    
    def _omega_reduce_cb(self,_):
        
        if self._tick_left == self._tick_left_last:
            # no change for about 20ms -> wheel is not spining or very slow
            self.omega_left = 0
            self.omega_window_left = []
            self.window_pointer_left = 0
        self._tick_left_last = self._tick_left
        if self._tick_right == self._tick_right_last:
            # no change for about 20ms -> wheel is not spining or very slow
            self.omega_right = 0
            self.omega_window_right = []
            self.window_pointer_right = 0        
        self._tick_right_last = self._tick_right            

    def left_enc_cb(self, tick_no) -> None:

        self._tick_left = tick_no

        if self._tick_left == 1:
            # This is the first tick, we do not know how fast are we
            self._last_tick_timing_left = rospy.get_time()
            self.omega_left = 0
        else:
            now     = rospy.get_time()
            delat_t = now - self._last_tick_timing_left
            self._last_tick_timing_left = now
            if self._tick_left > self._tick_left_last:
                _omega = (2*pi/self._resolution)/delat_t
                if abs(_omega) > 20:
                    _omega = 20
                self.stop_counter_left = 0
            elif self._tick_left < self._tick_left_last:
                _omega = (-2*pi/self._resolution)/delat_t
                if abs(_omega) > 20:
                    _omega = -20

           
            if len(self.omega_window_left) < self.window_length:
                self.omega_window_left.append(_omega)
            else:
                self.omega_window_left[self.window_pointer_left] = _omega
                self.window_pointer_left += 1
                if self.window_pointer_left > self.window_length - 1:
                    self.window_pointer_left = 0 

    def right_enc_cb(self,tick_no) -> None:
        
        self._tick_right = tick_no

        if self._tick_right == 1:
            # This is the first tick, we do not know how fast are we
            self._last_tick_timing_right = rospy.get_time()
            self.omega_right = 0
        else:
            now     = rospy.get_time()
            delat_t = now - self._last_tick_timing_right
            self._last_tick_timing_right = now
            if self._tick_right > self._tick_right_last:
                _omega = (2*pi/self._resolution)/delat_t
                if abs(_omega) > 20:
                    _omega = 20
            elif self._tick_right < self._tick_right_last:
                _omega = (-2*pi/self._resolution)/delat_t
                if abs(_omega) > 20:
                    _omega = -20
            
            
            if len(self.omega_window_right) < self.window_length:
                self.omega_window_right.append(_omega)
            else:
                self.omega_window_right[self.window_pointer_right] = _omega
                self.window_pointer_right += 1
                if self.window_pointer_right > self.window_length - 1:
                    self.window_pointer_right = 0 

    def _cb_publish(self,_):
        self.seq += 1
        
        if len(self.omega_window_left) > 0:
            self.omega_left = sum(self.omega_window_left)/len(self.omega_window_left)
            
        if len(self.omega_window_right) > 0:
            self.omega_right = sum(self.omega_window_right)/len(self.omega_window_right)
        
        now = rospy.Time.now()
        
        msg_to_send = WheelsEncoder()
        msg_to_send.header.seq = self.seq
        msg_to_send.header.stamp.secs   = now.secs
        msg_to_send.header.stamp.nsecs  = now.nsecs
        msg_to_send.omega_left  = self.omega_left
        msg_to_send.omega_right = self.omega_right
        msg_to_send.left_ticks  = self._tick_left
        msg_to_send.right_ticks = self._tick_right

        self.pub_omega.publish(msg_to_send)

    def shut_hook(self):
        del self.left_driver
        del self.right_driver

if __name__ == '__main__':

    try:
        rospy.init_node("wheel_encoders")
        N = WheelEncodersNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard Shutdown')