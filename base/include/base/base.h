/*
 * base.h
 *
 * Created on: 	Nov 17, 2013
 *     Author:	Floris Gaisser
 *
 * Versions:
 * 1.0		Initial version
 *  
 */

#ifndef BASE_H_
#define BASE_H_

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <base/BaseStatus.h>

/// Basic base controller class
/**
 * This class reads the following values from the parameter server:
 * \param ~/motor_port The topic name of the \c shared_serial node used for communication with the motor
 * \param ~/motor_config The name of the motor configuration XML file
 * \param ~/wheel_diameter The diameter of the wheels in [m]
 * \param ~/wheel_base The distance between the wheels in [m]
 */
class Base {
  protected:
	ros::NodeHandle		nh_;
	ros::Subscriber		pos_sub_;
	ros::Subscriber		vel_sub_;
	ros::Subscriber		pos_vel_sub_;
	ros::Subscriber		brake_value_;
	ros::Publisher		status_pub_;

	C3mxlROS*		left_motor_;
	C3mxlROS*		right_motor_;

	double			wheel_diameter_;
	double			wheel_base_;

	bool			mode_pos_;

  protected:

	/**
	 * Callback that handles positions
	 */
	void positionCallback(const geometry_msgs::Twist::ConstPtr &msg);

	/**
	 * Callback that handles velocities
	 */
	void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg);

	/**
	 * Callback that handles positions and turns them into velocities
	 */
	void positionVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
	
	void brakeCallback(const std_msgs::Float32::ConstPtr &msg);

	/**
	 * Publish the status of the base motors
	 */
	void publishStatus();

  public:
	Base() : nh_("~") {
	  init();
	}

	~Base() {
	  delete left_motor_;
	  delete right_motor_;

	  nh_.shutdown();
	}

	/**
	 * Initialize the base motors
	 * \note Called during construction
	 */
	void init();

	/**
	 * Await and process commands
	 */
	void spin();
};


#endif /* BASE_H_ */
