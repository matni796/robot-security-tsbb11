#!/bin/bash

#Script for starting up system in robot security system
#Remember that this terminal will act totally weird after running this script.
#To look at the specific output of one node run the them in different windows.

# a Start up ros system
roscore &
sleep 2

#1. Commands depending on the robot

#load udf file (3D-model of the robot?)
rosparam load $CAMERA_MATRIX_PATH/src/motoman/motoman_config/cfg/sia20D_mesh.xml  robot_description
#notice here that the environment Variable CAMERA_MATRIX_PATH is the path to your catkin_ws.
#If it is not set you have to set it, (google the export command)
#a bit odd name but what can I say... :p

#establishing connection to robot
roslaunch motoman_driver robot_interface_streaming_dx100.launch robot_ip:=192.168.255.1 &
sleep 2

# start the viewer
roslaunch sia20d mesh arm navigation planning scene warehouse viewer sia20d mesh.launch &


#2. Commands for starting up kinect related software

# b start up kinect stream
roslaunch freenect_launch freenect.launch &
sleep 2

# c run background-modelling
rosrun background_modelling background_modelling &
sleep 1

# d run clustering
rosrun clustering clustering &
sleep 1

# e run calibration
rosrun calibration calibration &
sleep 1

# f run distance calculator
rosrun distance_calc distance_calc &
sleep 1

