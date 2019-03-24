# csrobot_ros  

This sample shows a sample CoSpace field with two robots.  
The robots can talk and hear in ROS topic.  
A rgb camera, three soners, two color sensors are publishing their information.  
There is a sample world file as "worlds/csrobot_ros.world".  

## How to use the robot.  
You need three terminals.  

### At Terminal 1 :  
You can run this sample by using a following command.  

    $ roslaunch csrobot_ros spawn_csrobot_ros_match.launch   

### At Terminal 2 (Automatically appeared):  
You can move the "ROBOT_1" with an ordinary ROS teleop package. It will be run automatically.  

### At Terminal 3 (Automatically appeared):  
You can move the "ROBOT_2" with an ordinary ROS teleop package. It will be run automatically.  

## How to terminate.  
At the terminal 1, push the control key and the "c" key simultaneously, until the command prompt appear again.  

## The robot model and the plugins.  
|Model Name|Plugin(Program) Filename(s)|
|---|---|
|sdf/model.config, sdf/model.sdf|src/change_body_color.cc, src/change_led_color.cc|

Date : 24 Mar. 2019
