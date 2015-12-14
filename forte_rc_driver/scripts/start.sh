#!/bin/bash

xterm -hold -e "/opt/ros/indigo/bin/rosrun rosserial_python serial_node.py /dev/forteRC_arduino" &
sleep 1
roslaunch $HOME/catkin_ws/src/forte_rc/forte_rc_driver/launch/forte_rc_serial3_comm.launch &
sleep 3
roslaunch $HOME/catkin_ws/src/forte_rc/forte_rc_driver/launch/forte_rc_description_lasers.launch &
sleep 5
roslaunch $HOME/catkin_ws/src/forte_rc/forte_rc_driver/launch/laserscan_multi_merger.launch &
sleep 1
roslaunch $HOME/catkin_ws/src/forte_rc/forte_rc_driver/launch/pololu_servos.launch &
 
$SHELL

exit 0

