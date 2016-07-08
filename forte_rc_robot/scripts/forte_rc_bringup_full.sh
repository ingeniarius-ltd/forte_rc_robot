#!/bin/bash

gnome-terminal -e "/opt/ros/indigo/bin/roslaunch $HOME/catkin_ws/src/forte_rc_robot/forte_rc_robot/launch/bring_up_forterc.launch" 
sleep 3
gnome-terminal -e "/opt/ros/indigo/bin/roslaunch $HOME/catkin_ws/src/forte_rc_robot/forte_rc_robot/launch/lasers/laser_merger.launch"
gnome-terminal -e "/opt/ros/indigo/bin/roslaunch $HOME/catkin_ws/src/forte_rc_robot/forte_rc_robot/launch/teleop_wii_forterc.launch"
sleep 2
gnome-terminal -e "/opt/ros/indigo/bin/roslaunch $HOME/catkin_ws/src/forte_rc_robot/forte_rc_driver/launch/pololu_servos_moveit.launch"
sleep 1
gnome-terminal -e "/opt/ros/indigo/bin/roslaunch $HOME/catkin_ws/src/forte_rc_robot/forte_rc_2dnav/launch/automatica_dwa_forte_rc.launch"
sleep 1
gnome-terminal -e "/opt/ros/indigo/bin/roslaunch $HOME/catkin_ws/src/FORTE_ROSv10/launch/display.launch"
sleep 1
gnome-terminal -e "/opt/ros/indigo/bin/rosrun rviz rviz"
