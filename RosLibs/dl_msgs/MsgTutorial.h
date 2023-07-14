#ifndef _ROS_dl_msgs_MsgTutorial_h
#define _ROS_dl_msgs_MsgTutorial_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dl_msgs
{

  class MsgTutorial : public ros::Msg
  {
    public:
      typedef float _dlYaw_type;
      _dlYaw_type dlYaw;

    MsgTutorial():
      dlYaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_dlYaw;
      u_dlYaw.real = this->dlYaw;
      *(outbuffer + offset + 0) = (u_dlYaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dlYaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dlYaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dlYaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dlYaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_dlYaw;
      u_dlYaw.base = 0;
      u_dlYaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dlYaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dlYaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dlYaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dlYaw = u_dlYaw.real;
      offset += sizeof(this->dlYaw);
     return offset;
    }

    const char * getType(){ return "dl_msgs/MsgTutorial"; };
    const char * getMD5(){ return "abd5d1e9c3ac157a0df3ba27b65d3384"; };

  };

}
#endif