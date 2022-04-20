# Wall_Follower
ROS Noetic / Python3 / Gazebo / Turtlebot3

This project was intended just to practice my basic ros knowledge.
Custom messages are used for ROS Service and Action.
ROS Service is used for searching the wall.
ROS Action is used to feedback the distance moved and to return the odom readings as the result.
This tutorials regrad that ROS Noetic, Python3 and Turtlebot3 packages are already installed on the user's computer.

# Prepare the gazebo environment

Enter the following command lines on your terminal.

roscore

Open another terminal and launch the turtlebot3 empty world : 

cd 

cd catkin_ws

export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

「　Manully insert a wall from Gazebo UI.　」

# How to launch the code

Open a new terminal and enter the following command lines one by one.

cd 

cd catkin_ws

catkin_make

source devel/setup.bash

roslaunch wall_following my_launch_wall.launch
