#!/bin/bash

gnome-terminal -e "/opt/ros/indigo/bin/roslaunch $HOME/catkin_ws/src/forte_rc_robot/forte_rc_robot/launch/bring_up_forterc.launch" 
sleep 3
gnome-terminal -e "/opt/ros/indigo/bin/roslaunch $HOME/catkin_ws/src/forte_rc_robot/forte_rc_robot/launch/lasers/laser_merger.launch"

