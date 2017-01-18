# TUD_COOP_UV - TU Darmstadt Cooperative Unmanned Robots

## Introduction
TUD_COOP_UV is a ROS Package that consolidates the research into tracking and coverage of the Control Methods and Robotics Lab at TU Darmstadt. This package implements the nodes required for the tracking of a Robotino Robot by an AR.Drone 2.0 Quadcopter and eventually simultaneously perform the Coverage of the surrounding environment.

Currently this package is designed to work only with an Ardrone 2.0 quadcopter, but there are plans to generalize the package in order to be used with any custom quadcopter.




## External packages required

### ardrone_autonomy

This package can be found on: [ardrone_autonomy](http://github.com/AutonomyLab/ardrone_autonomy). It is required to control the Ardrone 2.0 with ROS. Can also be installed without compiling the source package, for example in ROS indigo:

```
sudo apt-get install ros-indigo-ardrone-autonomy
```

### ardrone_velocity

This package can be found on: [ardrone_velocity](http://github.com/raultron/ardrone_velocity). Ardrone Velocity is a ROS Package for PID Velocity control of the AR.DRone 2.0. It is designed to work together with the ardrone_autonomy package. Based on a velocity reference (given by the user) and a velocity measurement (obtained from ardrone_autonomy), this package implements a PID velocity controller of the Quadcopter. Since this controller depends on Wifi communication with the AR.Drone to obtain the velocity measurement, it won't be perfect due to delays in communication.

This package is needed if the references to be used for the tracking and coverage are velocities. It is important to remember that the ardrone_autonomy receives a geometry_msgs/Twist message in the topic /cmd_vel and those values are sent through wifi to the quadcopter using the Ardrone SDK. However the linear.x and linear.y portions of this message are actually the required tilting angles of the quadcopter and not velocities. Controlling tilting angles is similar to control the accelerations in x and y of the quadcopter.

Clone this repository into the src folder of an existing or new catkin compatible workspace, and then build it using catkin. For example, to compile in ROS Indigo:

```
cd ~/catkin_ws/src
git clone https://github.com/raultron/ardrone_velocity.git
cd ~/catkin_ws
rosdep install --from-paths src -i
catkin_make
```


### ar_sys

This package can be found on: [ar_sys](http://wiki.ros.org/ar_sys). It implementes a 3D pose estimation using ArUco marker boards. We use this package to detect the Aruco Marker boards on top of the ground robots using the images captured by the quadcopter camera. This package can be directly installed in Ubuntu:

```
sudo apt-get install ros-indigo-ar-sys
```


### tud_img_prep

This package can be found on: [tud_img_prep](http://github.com/raultron/tud_img_prep). It implements a set of OpenCV Image processing and Filtering tools in a dynamically reconfigured package. For more information check the package repository. This package improve the images captured by the quadcopter camera for a better marker recognition.

Clone this repository into the src folder of an existing or new catkin compatible workspace, and then build it using catkin. For example, to compile in ROS Indigo:

```
cd ~/catkin_ws/src
git clone https://github.com/raultron/tud_img_prep.git
cd ~/catkin_ws
rosdep install --from-paths src -i
catkin_make
```

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

## Demo launch file

#### Terminal 1
```
roscore
```

### Terminal 2
Launch de ardrone_autonomy driver (follow the instructions from the [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy) documentation:

```
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=0 _looprate:=400
```

### Terminal 3
```
roslaunch tud_coop_uv tracking_demo_bottom_ardrone.launch
```


## System description

## Flowchart
![Flowchart of the system](URL)

## Node description

### tracking

#### Subscribed topics
- /ardrone/odometry
  - message type: ```nav_msgs::Odometry```
  - description: this odometry information is published by the ardrone_autonomy package, it is necessary for the control.

#### Published topics
- /tracking/cmd_vel
  - message type: ```geometry_msgs::Twist```
  - description: this is the output twist reference that moves the quadcopter towards the center of the marker and aligns with it.
- /cmd_vel_marker
  - message type: ```visualization_msgs::Marker```
  - description: this marker can be used in RVIZ to visualize the current velocity reference command.

### Required transforms

- transform between ```/ardrone_base_link``` frame and ```/tracking_target``` frame.

The ```/ardrone_base_link``` is the coordinate frame located in the approximate center of the quadcopter. This frame is published by the ardrone_autonomy package.

The ```/tracking_target``` is the coordinate frame of the goal target that we want to track with the quadcopter, i.e the center of the marker.

In order to calculate the error in pose between the marker and the quadcopter we need a ROS transformation tree already in place (Check tf2 package). Ar_sys can publish automatically the transform between the camera frame and the ```/tracking_target``` if it is detected. And we finally need a fixed transformation between the camera frame and the center of the quadcopter.

In one demo launch file we use the bottom camera of the ardrone, the package ardrone_autonomy provides a transformation between the ```/ardrone_base_link``` and the ```/ardrone_base_bottomcam``` frame.

We also provide a demo launch file with an external camera attached to the quadcopter (higher quality than the integrated), in this case we have to manually publish a fix transformation in the launch file between the ```/ardrone_base_link``` and this new external ```/cam``` frame.

### Funcionality
This node uses the transformation information to implement a simple PID pose control on top of the marker at a given height. Its default tracking height above the marker is 1.1m and can be changed in the launch file by using the parameter: ```ref_quadcopter_height```.

And important assumption of the controller for simplification purposes is that the marker is parallel to the ground.

### merge

#### Subscribed topics
- /joy/cmd_vel
  - message type: ```geometry_msgs::Twist```
  - description: this is the twist velocity reference given by a joystik controlled by the user. The node joy_control publishes in this topic.
- /tracking/cmd_vel
  - message type: ```geometry_msgs::Twist```
  - description: this is the twist velocity reference provided by the tracking node
- /coverage/cmd_vel
  - message type: ```geometry_msgs::Twist```
  - description: this is the twist velocity reference provided by the (currently not available) coverage node

#### Published topics
- /cmd_vel_ref
  - message type: ```geometry_msgs::Twist```
  - description: this is the output reference twist velocity for the quadcopter

### Services
- /tud_coop_uv/set_controller
  - service type: ```tud_coop_uv::SetController```
  - description: by this service the joy_control node can select which one of the input cmd_vel messages are going to be passed to the quadcopter.

### Funcionality

This nodes merges velocity references from different sources. This allow the user to give direct velocity commands to the quadcopter using a joystick or to have a complete autonomous behavior using the references of the tracking controller. Also it is possible to select a combined control command which is the sum of all the three input topics.

### joy_control

#### Subscribed topics
- /joy
  - message type: ```sensor_msgs::Joy```
  - description: generic joystick message provided by the joy ROS node. It gives the status of all the buttons and axis of the joystick.

#### Published topics
- /joy/cmd_vel
  - message type: ```geometry_msgs::Twist```
  - description: this is the twist velocity reference given by a joystick controlled by the user. This is received by the merge node.
- /ardrone/takeoff
  - message type: ```std_msgs::Empty```
  - description: publishing here makes ardrone_autonomy send a takeoff command to the quadcopter. It is assigned to one of the joystick buttons.
- /ardrone/land
  - message type: ```std_msgs::Empty```
  - description: publishing here makes ardrone_autonomy send a land command to the quadcopter. It is assigned to one of the joystick buttons.
- /ardrone/reset
  - message type: ```std_msgs::Empty```
  - description: publishing here makes ardrone_autonomy send a reset command to the quadcopter. It is assigned to one of the joystick buttons.

#### Subscribed services
- /tud_coop_uv/set_controller
  - service type: ```tud_coop_uv::SetController```
  - description: by this service the joy_control node can select which one of the input cmd_vel messages are going to be passed to the quadcopter.

#### Button configuration for Logitech Wingman controller:

buttons
0 - A  --function: Enable/disbale flying with integrated hovering functionality (using the ardrone visual odometry for hovering)
1 - B  --function: Send landing command
2 - C  --function: Send takeoff command
3 - X  --function: Set controller as Joystick input only
4 - Y  --function: Set controller as Autonomous Tracking only
5 - Z  --function: Set controller as joint (sum of tracking and joy commands)
6 - L1  --function: Send reset command
7 - L2
8 - Start
9 - L2
10 - R2

Axes

0 - Left X   --function: twist_msg.linear.y
1 - Left Y   --function: twist_msg.linear.x
2 - Slider
3 - Right X  --function: twist_msg.angular.z (Rotation around Z)
4 - Right Y  --function: twist_msg.linear.z (actual velocity in Z)
5 - Right T
6 - Left and Right arrow
7 - Up and down arrow


#### Button configuration for Logitech Wireless Gamepad F710:

0 - A (green)  --function: Enable/disbale flying with integrated hovering functionality (using the ardrone visual odometry for hovering)
1 - B (red)  --function: Send landing command
2 - X (blue)  --function: Send takeoff command
3 - Y (Yellow) --function: Set controller as Joystick input only
4 - LB  --function: Set controller as Autonomous Tracking only
5 - RB  --function: Set controller as joint (sum of tracking and joy commands)
6 - back  --function: Send reset command
7 - start
8 -
9 - Left Joy press
10 - Right joy press

Axes

0 - Left X   --function: twist_msg.linear.y
1 - Left Y   --function: twist_msg.linear.
2 - Left T
3 - Right X  --function: twist_msg.angular.z (Rotation around Z)
4 - Right Y  --function: twist_msg.linear.z (actual velocity in Z)
5 - Right T
6 - Left and Right arrow
7 - Up and down arrow

### coverage
Not yet implemented.
