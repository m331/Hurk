#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class laserXY
{
public:
  laserXY();

private:
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser);
  
  ros::NodeHandle nh_;
  ros::Subscriber laser_sub_;
  ros::Publisher dth_pub_;
  
};

laserXY::laserXY()
{
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 10, &laserXY::laserCallback, this);
  dth_pub_ = nh_.advertise<geometry_msgs::Twist>("base/dth", 1);

}

void laserXY::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	int length = laser->ranges.size();

	double xy[length][2];
	double rmin = laser->range_min;
	double rmax = laser->range_max;
	double amin = laser->angle_min;
	double ainc = laser->angle_increment;
	
	double angle = amin;
	double dis = 0;
	
	double n = 0;
	double sumx = 0;
	double sumy = 0;
	double sumxx = 0;
	double sumxy = 0;
	
	for(int i = 0; i < length;i++){
		dis= laser->ranges[i];
		if(dis < rmax && dis > rmin){
			double x = dis * cos(angle);
			double y = dis * sin(angle);
			sumx += x;
			sumy += y;
			sumxx += x*x;
			sumxy += x*y;
			n++;
			//xy[i][0] = dis * cos(angle);
			//xy[i][1] = dis * sin(angle);			
		}
		else{
			//xy[i][0] = 0;
			//xy[i][1] = 0;
		}		
		angle = angle + ainc;		
	}
	double c;
	double d;
	double theta;
	if (n != 0) {
		c = (n*sumxx - sumx*sumx)/(n*sumxy - sumx*sumy);
		d = (1/n)*sumx - (c/n)*sumy;
		theta = atan(c);
	} else {
		d = 0;
		theta = 0;
	}
	
	geometry_msgs::Twist disth;
	//disth.angular.x = sumx; // test
	//disth.angular.y = sumy; // test
	disth.angular.z = theta;
	disth.linear.x = d;
	//disth.linear.z = n; //test
	
	dth_pub_.publish(disth);
	
	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_xy");
  laserXY laser_xy;

  ros::spin();
}
