#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

/*

Deze class laat de robot automatisch rijden aan de hand van gedetecteerde obstakels

*/


class Obstakels
{
public:
  Obstakels();

private:
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser);
  
  ros::NodeHandle nh_;
  ros::Subscriber laser_sub_;
  ros::Publisher vel_pub_;
  
};

Obstakels::Obstakels()
{
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 10, &Obstakels::laserCallback, this);
   vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/cmd_vel", 1);
}

void Obstakels::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	int length = laser->ranges.size();
	
	double maxdis = 2.0; //maximum distance objects are not ignored
	double miny = -0.42; //min. y-waarde voor het basisplatform in het assenstelsel van de kinect
	double maxy = 0.42;  //max. y-waarde voor het basisplatform in het assenstelsel van de kinect
	double stopdis = maxy - miny; //stopdistance
	
	
	double xy[length][2];
	double rmin = laser->range_min;
	double rmax = laser->range_max;
	double amin = laser->angle_min;
	double ainc = laser->angle_increment;
	
	double angle = amin;
	double dis = 0;
	
	double objx = 0;
	double objy = 0;
	double objr = maxdis;
	double objth = 0;
	
	for(int i = 0; i < length;i++){
		dis= laser->ranges[i];
		if(dis < rmax && dis > rmin){
			//double x = dis * cos(angle);
			//double y = dis * sin(angle);
			if (dis < objr) {
				objr = dis;
				objth = angle;
				//objx = x;
				//objy = y;			
			}
		}	
		angle = angle + ainc;		
	}
	
  geometry_msgs::Twist vel;
  
  if (objr < stopdis) {
	//er is een object te dichtbij, dus we gaan stilstaan en draaien totdat we weer kunnen rijden
	vel.angular.z = -0,3; //rechtsaf draaien
	vel.linear.x = 0.0;
  } else {
	if (objth < 0) {
		vel.angular.z = 0.5;
	} else {
		vel.angular.z = -0.5;
	}
	if (objr > 0.5 * maxdis) {
		vel.angular.z += 0.5 * objth;
	}
	vel.linear.x = 0.4;
  }

  ROS_INFO("x: %f z: %f", vel.linear.x, vel.angular.z);
  vel_pub_.publish(vel);


	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_xy");
  Obstakels laser_xy;

  ros::spin();
}
