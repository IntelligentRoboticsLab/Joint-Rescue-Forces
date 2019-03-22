# CoSpace_in_Gazebo_ROS
A prototype and rebuilt CoSpace environment in Gazebo-ROS.  

## REQUIREMENT OF THIS REPOSITORY

### Install ROS Kinetic and Gazebo8 from PPA
#### *[Ubuntu install of ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
#### *[Install Gazebo using Ubuntu packages](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=8.0)  
Do followings:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116  
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'  
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -  
    sudo apt-get update  
    sudo apt-get install -y cmake g++ protobuf-compiler pavucontrol libgazebo8 libgazebo8-dev ros-kinetic-desktop ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-image-view2 ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-message-to-tf ros-kinetic-tf2-geometry-msgs ros-kinetic-audio-common ros-kinetic-costmap-2d ros-kinetic-image-transport ros-kinetic-image-transport-plugins ros-kinetic-urdf-tutorial ros-kinetic-gazebo8-ros*  
    sudo rosdep init  
    rosdep update  
    sudo apt-get install -y python−rosinstall  
    gazebo (and wait for finish of downloading fundamental models)  

## Preparing for using this repository.  
Just do the next commands.  

    $ cd ~  
    $ git clone https://github.com/IntelligentRoboticsLab/Joint-Rescue-Forces.git  
    $ cd ~/Joint-Rescue-Forces  
    $ catkin_make  
    $ source setup.bash  

## How to run.  
Just do the next commands.  
You will be able to see a Gazebo Window and two new terminals.  
In the Gazebo window, there will be two robots and a sample CoSpace field.  
The Blue robot is "ROBOT_1", and the Red robot is "ROBOT_2".  
The terminals are for controlling each robot.  
Each robot has a camera, three soner sensors, two color sensors.  
Every input devices are publishing topics.  
The blue robot is subscribing a topic "/ROBOT_1/cmd\_vel", the red one is subscribing a topic "/ROBOT_1/cmd\_vel".  

    $ cd ~  
    $ cd ~/Joint-Rescue-Forces  
    $ source setup.bash  
    $ roslaunch csrobot_ros spawn_csrobot_ros_match.launch

## Location of models.
Robot models are under "\~/Joint-Rescue-Forces/src/csrobot\_ros/sdf-blue" directory and "\~/Joint-Rescue-Forces/src/csrobot\_ros/sdf-red" directory.  

Edit date: 22 Mar. 2019
