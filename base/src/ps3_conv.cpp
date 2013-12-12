#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>

class PS3Conv
{
public:
  PS3Conv();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  double linear_, angular_,a_scale_,l_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};

PS3Conv::PS3Conv():
  linear_(3),
  angular_(0),
  a_scale_(1.5),
  l_scale_(1)
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &PS3Conv::joyCallback, this);

}

void PS3Conv::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = pow(a_scale_*joy->axes[angular_],3);
  vel.linear.x = pow(l_scale_*joy->axes[linear_],3);
  ROS_INFO("x: %f z: %f", vel.linear.x, vel.angular.z);
  vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3_conv");
  PS3Conv ps3_conv;

  ros::spin();
}
