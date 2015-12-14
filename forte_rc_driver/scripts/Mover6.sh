#!/bin/bash
source ~/rosbuild_ws/setup.bash
export ROS_PACKAGE_PATH=$HOME/rosbuild_ws:$HOME/catkin_ws:$ROS_PACKAGE_PATH
xterm -hold -e "roslaunch $HOME/rosbuild_ws/cpr_rviz_plugin/cpr_mover6.launch" &

source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/catkin_ws:$ROS_PACKAGE_PATH
roslaunch $HOME/catkin_ws/src/forte_rc/forte_rc_driver/launch/cpr_mover6.launch
$SHELL

exit 0
