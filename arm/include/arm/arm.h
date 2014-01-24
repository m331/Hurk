#ifndef ARM_H_
#define ARM_H_

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <arm/ArmStatus.h>
#include <arm/Jolt4DOF.h>
//#include <jolt_common/MotorStatus.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <Eigen/Dense>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <arm/PredefPos.h>
#include <arm/SetHeight.h>


#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"

#include <exception>

class Arm
{
private:
	ros::NodeHandle 	nh_;
	ros::Subscriber 	vel_sub_;
	ros::Subscriber 	pos_sub_;
	ros::Subscriber 	predef_sub_;
	ros::Subscriber 	ikvel_sub_;
	ros::Publisher 		status_pub_;
	ros::Publisher 		joint_pub_;
	//ros::Publisher		gripper_sensor_pub_;
	ros::Publisher		bus_voltage_pub_;
	ros::ServiceServer	predef_pos_service_, set_height_service_;
	tf::TransformListener tf_listener_;

	C3mxlROS 			*shoulder_motor_, *elbow_motor_, *wrist_roll_motor_, *wrist_pitch_motor_;
	const static double default_speed_,default_accel_, wristpitch_factor_;
	const static double elbow_upper_limit_,elbow_lower_limit_,wrist_pitch_upper_limit_,wrist_pitch_lower_limit_;
	const static double shoulder_length_, elbow_length_, lambda_;
	//std_msgs::Float64 	gripper_sensor_data_, bus_voltage_;
	int 				ik_counter_;

	double				shoulder_abs_pos_, elbow_abs_pos_, wrist_pitch_abs_pos_, wrist_roll_abs_pos_;
	double				shoulder_rel_pos_, elbow_rel_pos_, wrist_pitch_rel_pos_;

protected:
	/// Called when a new velocity command is published
	/**
	 * Sends the new velocity to the neck motors
	 * \param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
	 */
	void velocityCallback(const arm::Jolt4DOF::ConstPtr &msg);

	/// Called when a new position command is published
	/**
	 * Sends the new velocity to the neck motors
	 * \param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
	 */
	void positionCallback(const arm::Jolt4DOF::ConstPtr &msg);

	/**
	 * Home callback
	 */
//	void predefCallback(const std_msgs::Int8::ConstPtr &msg);
	bool predefCallback(arm::PredefPos::Request &req, arm::PredefPos::Response &res);

	bool setheightCallback(arm::SetHeight::Request &req, arm::SetHeight::Response &res);


	/**
	 * Ik callback
	 * Takes dx and dz and converts it into th_shoulder and th_elbow
	 * Publishes the velocity obtained from inverse jacobian to the motors
	 */
	void  ikvelCallback(const geometry_msgs::Point::ConstPtr &msg);
	bool  setikVel(Eigen::Vector2f dX);

	/// Called at 30hz
	/**
	 * Sends the status of the neck motors
	 */
	void statusPublish();
	void motorControl();
	//void analogsensorPublish();

	//jolt_common::MotorStatus getStatus(C3mxlROS *motor);
	arm::ArmStatus status_msg_;

	arm::Jolt4DOF motor_command_;
	bool mode_pos_;


	void homePosition();
	void initablePosition();
	void navPosition();
	void rotateHandDown();
	void twistrotate();

public:
	Arm() : nh_("~")
	{
		init();
	}

	~Arm();

	/// Initialize the base motors
	/** \note Called during construction */
	void init();

	/// Await and process commands
	void spin();


};


#endif /* ARM_H_ */
