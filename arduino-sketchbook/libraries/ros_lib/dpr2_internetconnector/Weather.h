#ifndef _ROS_SERVICE_Weather_h
#define _ROS_SERVICE_Weather_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Empty.h"

namespace dpr2_internetconnector
{

static const char WEATHER[] = "dpr2_internetconnector/Weather";

  class WeatherRequest : public ros::Msg
  {
    public:
      std_msgs::Empty e;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->e.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->e.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return WEATHER; };
    const char * getMD5(){ return "294e8deabb2b175862c8a1e0560d461e"; };

  };

  class WeatherResponse : public ros::Msg
  {
    public:
      char * data;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_data = strlen( (const char*) this->data);
      memcpy(outbuffer + offset, &length_data, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_data;
      memcpy(&length_data, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    const char * getType(){ return WEATHER; };
    const char * getMD5(){ return "992ce8a1687cec8c8bd883ec73ca41d1"; };

  };

  class Weather {
    public:
    typedef WeatherRequest Request;
    typedef WeatherResponse Response;
  };

}
#endif
