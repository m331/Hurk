/*
 * wheel_odometry.h
 *
 *  Created on: Dec 20, 2011
 *      Author: machielbruinink
 */

#ifndef WHEEL_ODOMETRY_H_
#define WHEEL_ODOMETRY_H_

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <XMLConfiguration.h>
#include <threemxl/dxlassert.h>

class WheelOdometry
{
public:
	WheelOdometry() : nh_("~")
	{
		init();
	}
	~WheelOdometry()
	{
		delete left_motor_;
		delete right_motor_;

		nh_.shutdown();
	}

	void init();

	void publishOdometry();

private:
	ros::NodeHandle nh_;

	ros::Publisher odom_pub_;
    ros::Time current_time_, last_time_;

	C3mxlROS *left_motor_, *right_motor_;
	double wheel_diameter_, wheel_base_;

	tf::TransformBroadcaster odom_broadcaster_;

    double x_;
    double y_;
    double th_;
    double vx_;
    double vth_;
};

#endif /* WHEEL_ODOMETRY_H_ */
