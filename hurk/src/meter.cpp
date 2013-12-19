#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>

class Meter
{
public:
  Meter();

private:
  void distanceCallback(const geometry_msgs::Twist::ConstPtr& distance);
  
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  ros::Subscriber dis_sub_;
  
};

Meter::Meter(){
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/cmd_vel", 1);
  dis_sub_ = nh_.subscribe<geometry_msgs::Twist>("base/dth", 10, &Meter::distanceCallback, this);

}

void Meter::distanceCallback(const geometry_msgs::Twist::ConstPtr& distance)
{
  double d = distance->linear.x;
  double th = distance->angular.z;
  geometry_msgs::Twist vel;
  vel.angular.z = th/-2.0;
  vel.linear.x = (d - 1.0)/2.0;
  
  if(vel.linear.x > 1.0)
	vel.linear.x = 1.0;
	
  if(vel.linear.x < -1.0)
	vel.linear.x = -1.0;

  if(vel.angular.z > 1.0)
	vel.angular.z = 1.0;

  if(vel.angular.z < -1.0)
	vel.angular.z = -1.0;

  ROS_INFO("x: %f z: %f", vel.linear.x, vel.angular.z);
  vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "meter");
  Meter meter;

  ros::spin();
}
