#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class Teleopforte_rc{
  
public:
  Teleopforte_rc();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  double v1_linear, v2_linear, v1_angular, v2_angular;
  double v_linear_, v_angular_;
  bool operation_mode = true;
  bool safety;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};


Teleopforte_rc::Teleopforte_rc():
  v1_linear(0.25),
  v2_linear(0.40),
  v1_angular(0.35),
  v2_angular(0.60),
  safety(true)
{
  nh_.param("v1_linear", v1_linear, v1_linear);
  nh_.param("v2_linear", v2_linear, v2_linear);
  nh_.param("v1_angular", v1_angular, v1_angular);
  nh_.param("v2_angular", v2_angular, v2_angular);
  nh_.param("safety", safety, safety);
  
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);  
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy_throttle", 10, &Teleopforte_rc::joyCallback, this);  //throttle topic pubs w/10Hz (wiimote uses 100Hz)
}


void Teleopforte_rc::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

  geometry_msgs::Twist vel;

  if(joy->buttons[0]) {         // button 1
      operation_mode = 1;

  } else if (joy->buttons[1]) { // button 2
      operation_mode = 0;
  }

  if (operation_mode){
      v_linear_ = v1_linear;
      v_angular_= v1_angular;
  } else {
      v_linear_ = v2_linear;
      v_angular_= v2_angular;
  }


  if((joy->buttons[3] + !safety)){      // button B

      vel.linear.x = -v_linear_ * joy->buttons[9];  // -Vx
      if (vel.linear.x==0.0){
        vel.linear.x = v_linear_ * joy->buttons[8];  // +Vx
      }

      vel.linear.y = -v_linear_ * joy->buttons[4];  // -Vy
      if (vel.linear.y==0.0){
        vel.linear.y = v_linear_ * joy->buttons[5];  // +Vy
      }

      vel.angular.z = -v_angular_ * joy->buttons[7]; // -W
      if (vel.angular.z==0.0){
        vel.angular.z = v_angular_ * joy->buttons[6];  // +W

      }
  }

  	
  vel_pub_.publish(vel);

}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "forte_rc_teleop");
  Teleopforte_rc Teleopforte_rc;

  ros::spin();
}
