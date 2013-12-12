/*
 * base.cpp
 *
 * Created on: 	Nov 17, 2013
 *     Author:	Floris Gaisser
 *
 * Versions:
 * 1.0		Initial version
 *  
 */

#include <XMLConfiguration.h>
#include <threemxl/dxlassert.h>
#include <base/base.h>

#define CLIP(x, l, u) ((x)<(l))?(l):((x)>(u))?(u):(x)

float multi = 1.0;

void Base::init() {
	ROS_INFO("Initializing base");

	// Read parameters
	std::string motor_port_name, motor_config_name;

	ROS_ASSERT(nh_.getParam("motor_port", motor_port_name));
	ROS_ASSERT(nh_.getParam("motor_config", motor_config_name));
	ROS_ASSERT(nh_.getParam("wheel_diameter", wheel_diameter_));
	ROS_ASSERT(nh_.getParam("wheel_base", wheel_base_));

	// Subscript to command topic
	pos_sub_ = nh_.subscribe("cmd_pos", 1, &Base::positionCallback, this);
	vel_sub_ = nh_.subscribe("cmd_vel", 1, &Base::velocityCallback, this);
	pos_vel_sub_ = nh_.subscribe("cmd_pos_vel", 1, &Base::positionVelocityCallback, this);
	brake_value_ = nh_.subscribe("/slow_down", 1, &Base::brakeCallback, this);
	
	status_pub_ = nh_.advertise<base::BaseStatus>("status",1);


	// Load motor configuration
	CXMLConfiguration motor_config_xml;
	ROS_ASSERT(motor_config_xml.loadFile(motor_config_name));

	CDxlConfig motor_config_left;
	motor_config_left.readConfig(motor_config_xml.root().section("left"));
	left_motor_ = new C3mxlROS(motor_port_name.c_str());
	left_motor_->setConfig(&motor_config_left);

	// Initialize left motor
	ros::Rate init_rate(1);
	while (ros::ok() && left_motor_->init() != DXL_SUCCESS) {
		ROS_WARN_ONCE("Couldn't initialize left wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	CDxlConfig motor_config_right;
	motor_config_right.readConfig(motor_config_xml.root().section("right"));
	right_motor_ = new C3mxlROS(motor_port_name.c_str());
	right_motor_->setConfig(&motor_config_right);

	// Initialize right motor
	while (ros::ok() && right_motor_->init() != DXL_SUCCESS) {
		ROS_WARN_ONCE("Couldn't initialize right wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	DXL_SAFE_CALL(left_motor_->set3MxlMode(motor_config_left.m3mxlMode));
	DXL_SAFE_CALL(right_motor_->set3MxlMode(motor_config_right.m3mxlMode));

	mode_pos_ = false;

	ROS_INFO("Base initialized");
}

void Base::spin() {
	ROS_INFO("Spinning");

	ros::Rate r(100);

	while(ros::ok()) {
		ros::spinOnce();
		publishStatus();
		r.sleep();
	}
}

/**
 * Callback that handles positions
 */
void Base::positionCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	// Base is nonholonomic, warn if sent a command we can't execute
	if (msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y) {
		ROS_WARN("I'm afraid I can't do that, Dave.");
		return;
	}

	if(!mode_pos_) {
		left_motor_->set3MxlMode(POSITION_MODE);
		left_motor_->get3MxlMode();

		right_motor_->set3MxlMode(POSITION_MODE);
		right_motor_->get3MxlMode();

		mode_pos_ = true;
	}
	
	double pos_linear  = msg->linear.x;
	double pos_angular = msg->angular.z;
	
	double conv_ang = pos_angular * ((4*atan(1))/180);
	
	double x = pos_linear / (wheel_diameter_/2);
	double th = conv_ang * (wheel_base_/wheel_diameter_);
	
	left_motor_->setPos(x - th);
	right_motor_->setPos(x + th);
	
}

/**
 * Callback that handles velocities
 */
void Base::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	// Base is nonholonomic, warn if sent a command we can't execute
	if (msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y) {
		ROS_WARN("I'm afraid I can't do that, Dave.");
		return;
	}

	if(mode_pos_) {
		left_motor_->set3MxlMode(SPEED_MODE);
		left_motor_->get3MxlMode();

		right_motor_->set3MxlMode(SPEED_MODE);
		right_motor_->get3MxlMode();

		mode_pos_ = false;
	}
	
	double vel_linear  = msg->linear.x/(wheel_diameter_/2);
	double vel_angular = msg->angular.z * (wheel_base_/wheel_diameter_);

	double vel_left    = vel_linear - vel_angular;
	double vel_right   = vel_linear + vel_angular;
	ROS_INFO("%f", multi);
	left_motor_->setSpeed(multi * vel_left);
	right_motor_->setSpeed(multi * vel_right);
}

/**
 * Callback that handles positions and turns them into velocities
 */
void Base::positionVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	// Base is nonholonomic, warn if sent a command we can't execute
	if (msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y) {
		ROS_WARN("I'm afraid I can't do that, Dave.");
		return;
	}

	if(mode_pos_) {
		left_motor_->set3MxlMode(SPEED_MODE);
		left_motor_->get3MxlMode();

		right_motor_->set3MxlMode(SPEED_MODE);
		right_motor_->get3MxlMode();

		mode_pos_ = false;
	}
	
	double pos_linear  = msg->linear.x;
	double pos_angular = msg->angular.z;
	
	double conv_ang = pos_angular * ((4*atan(1))/180);
	
	double x = pos_linear / (wheel_diameter_/2);
	double th = conv_ang * (wheel_base_/wheel_diameter_);
	
	
	
	left_motor_->setSpeed(multi*(x - th));
	right_motor_->setSpeed(multi*(x + th));
	
	//TO DO
	//STOPPEN NA X aantal meters
}

void Base::brakeCallback(const std_msgs::Float32::ConstPtr &msg) {
	multi = msg->data;
	
}



/**
 * Publish the status of the base motors
 */
void Base::publishStatus() {
	base::BaseStatus msg;
	
	left_motor_->getState();
	right_motor_->getState();
	left_motor_->getStatus();
	right_motor_->getStatus();

	msg.left.speed = left_motor_->presentSpeed();
	msg.right.speed = right_motor_->presentSpeed();
	
	msg.left.position = left_motor_->presentPos();
	msg.right.position = right_motor_->presentPos();
	
	msg.left.torque = left_motor_->presentTorque();
	msg.right.torque = right_motor_->presentTorque();
	
	msg.left.status = left_motor_->presentStatus();
	msg.right.status = right_motor_->presentStatus();

	status_pub_.publish(msg);

}

/**
 * Start the engines, ready, set,... go!
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "base");

	Base base;
	base.spin();

	return 0;
}



