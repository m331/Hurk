#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <gripper/DxlClass.h>
#include <threemxl/C3mxlROS.h>

/* This node receives a force message, then calculates
 * the required current of the motor in the gripper to
 * reach this force, and then it sends this command to
 * the gripper. Much code is borrowed from 'example.cpp'
 * in threemxl package.
 */
 
#define FORCE_TO_CURRENT 1

void DxlClass::init(char *path)
{
  CDxlConfig *config = new CDxlConfig();

  if (NULL)// Can be used in case of using shared serial.
  {
    ROS_INFO("Using shared_serial");
    motor_ = new C3mxlROS(path);
  }
  else
  {
    ROS_INFO("Using direct connection");
    motor_ = new C3mxl();
    
    serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
    serial_port_.set_speed(LxSerial::S921600);
    motor_->setSerialPort(&serial_port_);
  }

  motor_->setConfig(config->setID(111));
  motor_->init(false);
  motor_->set3MxlMode(CURRENT_MODE);

  delete config;
}

void DxlClass::setgripcurrent(double current)
// Sends current in A to 3mxel.
{
	ROS_DEBUG("setgripcurrent called");
	motor_->setCurrent(-0.1);
}



void DxlClass::incoming(const std_msgs::Float32::ConstPtr &force)
// Gets called when message is received on channel.
{
	ROS_DEBUG("incoming called");
	
	// 'data' is the float in the std_msgs::Float32 message class.
	double current = FORCE_TO_CURRENT * force->data;
	// Using -> with a pointer is the same as using . with a normal variable
	
	setgripcurrent(current);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gripcontrol");
	
	DxlClass dxl_ros_example;
	
	// ROS subscribe to topic
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("gripforce", 1000, &DxlClass::incoming, &dxl_ros_example);
	// Using a class method as callback requires this special notation (more info: http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks)
	
  
  dxl_ros_example.init(NULL);
  //dxl_ros_example.spin();
  
  
  ros::spin();
  
  return 0;   
} 
