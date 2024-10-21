# VPA Robot Car Interface

The package of VPA Car interface aims to contain all sensors, actuators drivers in one package. This package is therefore free of any high-level logic but only performs commands from other sensors.

This package is again branched by the type of robot.

# DB19

The DB19 chassis is originally a duckietwon db19 chassis. Extra tof sensors and closed loop speed controller is implemented based on its coding. On the other hand, despite the well-developed package [here]|(https://github.com/duckietown/dt-duckiebot-interface), the dependencies are across different packages. The VPA MiniCCAM builds a lightweight package for this chassis for entry-level students.
## Dependencies
- [ ] containerization of this package

Currently, the dependencies still require manual installment. The package is built based on ROS Noetic, which is python3. The OS of the current VPA db19 robot is Ubuntu-mate 20.04 upgraded from 18.04. This is done this way since the 20.04 image is broken.

The hardware of the db19 robot is Raspberry Pi 3B+. The camera and I2C interface is required to be enabled.

## Sensors
The sensor of this chassis including
1. Encoders of each motor, but only one phase of output is transmitted to the Pi. Limited by the Duckietown HAT hardware settings.
2. A 170-degree wide-range camera on the CSI port.
3. A VL51L1x ToF sensor on I2C bus.
4. A GY521 (MPU6050) IMU

## Actuators
The two motors are driven by the Duckietown HAT, controlled through I2C bus as well. There are originally four RGB LEDs that may be used for somewhat purpose, but it is now disconnected due to lack of usage.

## Quick Start

### start traction
The wheels of the vehicle are subscribing to <code>cmd_vel</code> of [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html). The wheel speed is by default based on a PI controller based on encoder readings.

To start, run
```
roslaunch vpa_robot_interface vpa_traction_start.launch
```
> [!NOTE]
> The wheel dirver node subscribe to a <code>/global_brake</code> of <code>std_msgs/Bool</code>. It is by default True. It requires a control node to allow the operation for safty reasons in the lab
- [ ] feedback from IMU sensor
### camera
To start the camera driver, which is based on [usb_cam](http://wiki.ros.org/usb_cam), you may run
```
roslaunch vpa_robot_interface vpa_camera.launch
```
By default, it is set to 320*240 in size for higher fps, which is 10 fps. It is configurable by setting the args of the launch file. We also recommend to do instrinstic calibration before using by
```
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.031 image:=robot_cam/image_raw camera:=robot_cam
```
on a remote machine and overwrite its current calibration file at <code>config</code>.
This calibration pattern is based on the ones that are available at the MiniCCAM lab, please modify accordingly if needed.
### ToF
The applied TOF400F sensor is based on [vl53l1x](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html) and its API at this [link](https://github.com/pimoroni/vl53l1x-python).

To start
```
roslaunch vpa_robot_interface vpa_tof.launch
```
- [ ] understand & resolve why the distance measurement sometimes do not bounce back when obstacles are removed
### IMU
The applied IMU sensor is based on [MPU6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) on I2C bus.
It is publishing two topics
- “imu": [sensor_msgs/Imu Message](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) at 20Hz by default and configurable
- "Temperature": [sensor_msgs/Temperature Message](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Temperature.html) at 1Hz
```
roslaunch vpa_robot_interface vpa_tof.launch
```

