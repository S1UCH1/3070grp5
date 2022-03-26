# 3070grp5

This is the ROBOCON 3070 Group 5 project repository.



## Basic info

The ROS package is responsible for subscribing joystick data, converting and publishing RPM speeds to the Arduino program.
The `grp5_node` will create 4 seperate `Float32` channels of name
```
whlspd1
whlspd2
whlspd3
whlspd4
```
of which is subscribed by `Rm_motor_speed_control`.

## Build

```
# While in ../3070grp5/ros_workspace
colcon build
```

## Launch

Simply run `launch_script.bash`.
The script will launch `joy_node`, `grp5_node` and initialize the DDS-Agent needed for the arduino.
