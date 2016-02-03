# TUD_COOP_UV - TU Darmstadt Cooperative Unmanned Robots

## Introduction
TUD_COOP_UV is a ROS Package that consolidates the research into tracking and coverage of the Control Methods and Robotics Lab at TU Darmstadt. This package implements the nodes required for the tracking of a Robotino Robot by an AR.Drone 2.0 Quadcopter and simultaneously performs the Coverage of the surrounding environment.

## Nodes required for the system

### ar_sys_prep

This node can be found in the ar_sys_prep package, implements a Gaussian Filtering of the image obtained from the Ardrone Autonomy Driver (from the bottom camera of the quadcopter) for a better detection of the Aruco Board by the ar_sys main node.

subscribed topics: 
* /ardrone/bottom/image_raw
* /ardrone/bottom/camera_info

published topics: 
* /ardrone/bottom/filtered/image_raw

### ar_sys

### tracking

### merge

### coverage

### joy_control


## Dependencies

* An AR.Drone 2.0 Quadcopter
* A ground robot with and Aruco Board Marker on top
* The ROS package ardrone_autonomy (communication with AR.Drone 2.0)
* The ROS package ardrone_velocity (velocity control of the AR.Drone 2.0)
* The ROS package ar_sys (Detection of the Aruco Markers)



## Installation 
### Compile from source
This ROS package depends on the following ROS packages for compilation: geometry_msgs, roscpp, rospy, nav_msgs,std_srvs, std_msgs, tf, tf2, cgal.

Clone this repository into the src folder of an existing or new catkin compatible workspace, and then build it using catkin. For example, to compile in ROS Indigo:

```
cd ~/catkin_ws/src
git clone https://raultron@bitbucket.org/raultron/tud_coop_uv.git
cd ~/catkin_ws
rosdep install --from-paths src -i
catkin_make
```

## Usage
### Terminal 1
```
roscore
```

### Terminal 2
```
roscore
```

### Terminal 3
```
roscore
```

### Terminal 4
```
roscore
```
Launch de ardrone_autonomy driver (follow the instructions from the [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy) documentation:

```
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=0 _looprate:=400
```

Then run the ardrone_velocity main PID controller node:

```
rosrun ardone_velocity pid_control
```

With this setup the pid_control node will subscribe to the odometry sensor information provided by ardrone_autonomy and to the topic cmd_vel_pid (twist_msg) where it will receive the velocity reference. The controled velocity will be published to the cmd_vel topic of the ardrone_autonomy driver where the final velocity command will be sent to the quadrotor.