#!/usr/bin/python3

import VL53L1X
import time
import signal

class ToFVL53L1X(object):

    def __init__(self, address,bus_num):
        self.address = address
        # 1 = Short Range, 2 = Medium Range, 3 = Long Range 
        self.range = 2
        self.tof = VL53L1X.VL53L1X(i2c_bus=bus_num, i2c_address = self.address)
        self.start_sensor()
        
        self.my_roi = VL53L1X.VL53L1xUserRoi(tlx=3,tly=9,brx=6,bry=6)
        self.tof.set_user_roi(self.my_roi)
        
    def start_sensor(self):
        time.sleep(0.2)

        self.tof.open()
        self.tof.stop_ranging()
        time.sleep(0.2)
        self.tof.start_ranging(0)
        self.tof.set_timing(48000,100)
        print('current timing budget: ',self.tof.get_timing())

    def stop_sensor(self):
        self.tof.stop_ranging()
        self.tof.close()

    def get_distance(self):
        distance = 0.0
        distance = self.tof.get_distance() * 0.001 # mm to m conversion
        #print(distance)
        if distance > 0:
            return distance
        else:
            if distance == -1.185:
                self.stop_sensor()
                time.sleep(0.1)
                self.start_sensor()
            return -1

if __name__ == '__main__':
    T = ToFVL53L1X(address=0x29,bus_num=7)
    signal.signal(signal.SIGINT, T.stop_sensor)
    while True:
        distance_mm = T.get_distance()
        if distance_mm < 0:
            # Error -1185 may occur if you didn't stop ranging in a previous test
            print("Error: {}".format(distance_mm))
        else:
            print("Distance: {}m".format(distance_mm))
        time.sleep(0.1)