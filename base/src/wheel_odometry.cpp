#include <base/wheel_odometry.h>

#define ONLY_WHEEL

void WheelOdometry::init()
{
  ROS_INFO("Initializing wheel odometry");

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

  // Read parameters
  std::string motor_port_name, motor_config_name;

  ROS_ASSERT(nh_.getParam("motor_port", motor_port_name));
  ROS_ASSERT(nh_.getParam("motor_config", motor_config_name));
  ROS_ASSERT(nh_.getParam("wheel_diameter", wheel_diameter_));
  ROS_ASSERT(nh_.getParam("wheel_base", wheel_base_));

  // Load motor configuration
  CXMLConfiguration motor_config_xml;
  ROS_ASSERT(motor_config_xml.loadFile(motor_config_name));

  CDxlConfig motor_config_left;
  motor_config_left.readConfig(motor_config_xml.root().section("left"));
  left_motor_ = new C3mxlROS(motor_port_name.c_str());
  left_motor_->setConfig(&motor_config_left);

  // Initialize left motor
  ros::Rate init_rate(1);
  while (ros::ok() && left_motor_->init() != DXL_SUCCESS)
  {
    ROS_WARN_ONCE("Couldn't initialize left wheel motor, will continue trying every second");
    init_rate.sleep();
  }

  CDxlConfig motor_config_right;
  motor_config_right.readConfig(motor_config_xml.root().section("right"));
  right_motor_ = new C3mxlROS(motor_port_name.c_str());
  right_motor_->setConfig(&motor_config_right);

  // Initialize right motor
  while (ros::ok() && right_motor_->init() != DXL_SUCCESS)
  {
    ROS_WARN_ONCE("Couldn't initialize right wheel motor, will continue trying every second");
    init_rate.sleep();
  }

  current_time_ = ros::Time::now();
  last_time_ = ros::Time::now();
}

// reads the current change in wheel and publish as odometry
void WheelOdometry::publishOdometry()
{
	while(ros::ok())
	{
		left_motor_->getState();
		left_motor_->getStatus();
		right_motor_->getState();
		right_motor_->getStatus();

		double left = left_motor_->presentSpeed();
		double right = right_motor_->presentSpeed();

		vx_ = (wheel_diameter_ / 4) * (left + right);
		vth_ = ((wheel_diameter_ / 2) / wheel_base_) * (right - left);

	    current_time_ = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time_ - last_time_).toSec();
		double delta_x = (vx_ * cos(th_)) * dt;
		double delta_y = (vx_ * sin(th_)) * dt;
		double delta_th = vth_ * dt;

		x_ += delta_x;
		y_ += delta_y;
		th_ += delta_th;

	    //since all odometry is 6DOF we'll need a quaternion created from yaw
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

#ifdef ONLY_WHEEL
	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = current_time_;
	    odom_trans.header.frame_id = "odom";
	    odom_trans.child_frame_id = "base_link";

	    odom_trans.transform.translation.x = x_;
	    odom_trans.transform.translation.y = y_;
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = odom_quat;

	    //send the transform
	    odom_broadcaster_.sendTransform(odom_trans);
#endif

	    //next, we'll publish the odometry message over ROS
	    nav_msgs::Odometry odom;
	    odom.header.stamp = current_time_;
	    odom.header.frame_id = "odom";
	    odom.child_frame_id = "base_link";

	    //set the position
	    odom.pose.pose.position.x = x_;
	    odom.pose.pose.position.y = y_;
	    odom.pose.pose.position.z = 0.0;
	    odom.pose.pose.orientation = odom_quat;

	    //set the velocity
	    odom.twist.twist.linear.x = vx_;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.angular.z = vth_;

	    //publish the message
	    odom_pub_.publish(odom);

	    last_time_ = current_time_;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_odometry");

  WheelOdometry odom;
  odom.publishOdometry();

  return 0;
}
