#!/usr/bin/python3

import rospy

from tof_drivers.tof400f_usart import ToF400F

from sensor_msgs.msg import Range

class ToFDriverNode:

    def __init__(self) -> None:
        
        rospy.on_shutdown(self.shut_hook)
        self.tof = ToF400F()

        self.veh_name         = rospy.get_namespace().strip("/")
        if len(self.veh_name) == 0:
            self.veh_name = 'db19'

        self._timer       = rospy.Timer(rospy.Duration(1/10), self._read_data)
        self._publish_res = rospy.Timer(rospy.Duration(1/5), self._publish_data)

        self._pub_tof     = rospy.Publisher('/tof_distance', Range, queue_size=1)

        self.tof_distance = 5

        rospy.loginfo("%s: tof sensor ready",self.veh_name)

    def _read_data(self, _):

        value = self.tof.get_distance()
        
        if value != -1 and value != None:
            self.tof_distance = value * 0.01 # convert to meters
            if self.tof_distance > 1.36:
                self.tof_distance = 1.36
        
    def _publish_data(self, _):

        r = Range()

        r.header.stamp      = rospy.Time.now()
        r.header.frame_id   = '/tof_sensor'
        r.radiation_type    = Range.INFRARED
        r.field_of_view     = (27 / 180) * 3.14
        r.min_range         = 0.05
        r.max_range         = 1.36
        r.range             = self.tof_distance

        self._pub_tof.publish(r)
    
    def shut_hook(self):
        del self.tof.serial

if __name__ == '__main__':

    try:
        rospy.init_node("tof_sensor")
        N = ToFDriverNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard Shutdown')

    