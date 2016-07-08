#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_pololu_servo/MotorCommand.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

float vx = 0;
float vy = 0;
const float PI = 3.14159;
int zero_state_cnt = 0;
float smooth_angle[5] = {0};
const float head_offset = -90.0;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel){

     vx = cmd_vel->linear.x;
     vy = cmd_vel->linear.y;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "head_controller");
  ros::NodeHandle n;

  ros::Publisher pololu_servo_pub = n.advertise<ros_pololu_servo::MotorCommand>("pololu/command",1);
  ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 10, cmdVelReceived);
  //ros::Subscriber sub_cmd_vel = n.subscribe("test_vel", 10, cmdVelReceived);

  ros::Rate loop_rate(5);

  ROS_INFO("Forte RC head ready to receive cmd_vel...");

  while (ros::ok())
  {

      // velocities in regard to the robot's axis
      //float vx = 0.5, vy = 0.0;

      float angle = atan2(vy,vx)*180.0/PI;

      if (fabs(angle) > 90.0)
          angle = (fabs(angle)/angle) * 180.0 - angle;

      // do not rotate more than 20%
      if (angle > 20.0)
          angle = 20.0;

      if (angle < -40.0)
          angle = -40.0;

      float rad_angle = ((head_offset + angle)*PI)/180.0;

      ros_pololu_servo::MotorCommand msg;

      // Average rolling window
      smooth_angle[0]=smooth_angle[1];
      smooth_angle[1]=smooth_angle[2];
      smooth_angle[2]=smooth_angle[3];
      smooth_angle[3]=smooth_angle[4];
      smooth_angle[4]=rad_angle;

      for(int i=0; i<4; i++){
         rad_angle += smooth_angle[i];
      }
      rad_angle = rad_angle/5.0;


      msg.joint_name="neck_yaw_joint";
      msg.speed=0.005;
      msg.acceleration=0.005;
      msg.position=rad_angle;



      if(rad_angle != 0.0 || zero_state_cnt==0){
          //ROS_INFO("%f",rad_angle);
          pololu_servo_pub.publish(msg);
          zero_state_cnt=0;
      }

      if(rad_angle == 0.0){
          zero_state_cnt++;
      }

      ros::spinOnce();
      loop_rate.sleep();
  }


  return 0;
}
