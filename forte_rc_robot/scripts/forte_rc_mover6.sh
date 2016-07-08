#!/bin/bash
source ~/rosbuild_ws/setup.bash
export ROS_PACKAGE_PATH=home/nuc/rosbuild_ws:home/nuc/catkin_ws:$ROS_PACKAGE_PATH
xterm -hold -e "roslaunch /home/nuc/rosbuild_ws/cpr_rviz_plugin/cpr_mover6.launch" &

source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=home/nuc/catkin_ws:$ROS_PACKAGE_PATH
cd /home/nuc/catkin_ws/src/cpr_mover
rosrun cpr_mover cpr_mover
exit 0
