#ifndef __DXL_CLASS_H
#define __DXL_CLASS_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <CDxlGroup.h>

#include <std_msgs/Float32.h>

/// Usage example for CDynamixel and CDynamixelROS
class DxlClass
{
  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor_;   ///< Motor interface
    CDxlGroup *motors_;    ///< Multiple motor interface
    LxSerial serial_port_; ///< Serial port interface

  public:
    /// Constructor
    DxlClass() : nh_("~"), motor_(NULL) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlClass()
    {
      if (motor_)
        delete motor_;
      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    
    /// Spin
    /** Alternatively drives the motor clockwise and counterclockwise */
    void setgripcurrent(double current);
    
    void incoming(const std_msgs::Float32::ConstPtr &force);
};    

#endif /* __DXL_CLASS_H */
