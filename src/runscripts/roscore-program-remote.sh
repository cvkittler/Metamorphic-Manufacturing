#!/bin.bash

source /home/pi/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.100.104:11311
export ROS_IP=10.42.0.123
rosrun mmmqp_eoat mmmqp_eoat_node
read line