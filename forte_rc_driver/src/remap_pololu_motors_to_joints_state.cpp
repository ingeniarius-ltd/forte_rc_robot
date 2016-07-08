#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv){
    ros::init(argc,argv,"pololu_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states",10);
    ros::Rate lr(10);

    sensor_msgs::JointState msg;
    msg.name.push_back("jointHead1");
    msg.position.push_back(0.0);
    //msg.velocity.push_back(0.0);
    //msg.effort.push_back(0.0);

    msg.name.push_back("jointHead2");
    msg.position.push_back(0.0);
    //msg.velocity.push_back(0.0);
    //msg.effort.push_back(0.0);

    while(ros::ok()){
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        ros::spinOnce();
    }
}
