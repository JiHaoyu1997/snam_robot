#!/usr/bin/env python3
# this script based on: https://github.com/OSUrobotics/mpu_6050_driver/blob/master/scripts/imu_node.py 

import smbus
import socket
import rospy
import numpy as np

from sensor_msgs.msg import Temperature, Imu
from std_msgs.msg import Bool

from tf.transformations import quaternion_about_axis
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None

# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)


def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
    
    # Calculate a quaternion representing the orientation
    accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    # Read the gyro vals
    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
    
    # Load up the IMU message
    o = imu_msg.orientation
    o.x, o.y, o.z, o.w = orientation

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.header.stamp = rospy.Time.now()

    imu_pub.publish(imu_msg)

def signal_shut(msg:Bool):
    if msg.data:
        rospy.signal_shutdown('IMU sensor node shutdown')


temp_pub = None
imu_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')
    rospy.Subscriber("robot_interface_shutdown", Bool,signal_shut)
    robot_name = socket.gethostname()
    bus     = smbus.SMBus(rospy.get_param('~bus', 6))
    ADDR    = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)
    
    publish_freq    = rospy.get_param('~imu_pub_freq', 20)
    IMU_FRAME       = rospy.get_param('~imu_frame', 'imu_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    temp_pub    = rospy.Publisher('temperature', Temperature,queue_size=1)
    imu_pub     = rospy.Publisher('imu', Imu,queue_size=1)
    imu_timer   = rospy.Timer(rospy.Duration(1/20), publish_imu)
    temp_timer  = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.loginfo("%s: imu sensor ready",robot_name)
    rospy.spin()