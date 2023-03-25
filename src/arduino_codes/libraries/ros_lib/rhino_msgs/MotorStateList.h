#ifndef _ROS_rhino_msgs_MotorStateList_h
#define _ROS_rhino_msgs_MotorStateList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rhino_msgs/MotorState.h"

namespace rhino_msgs
{

  class MotorStateList : public ros::Msg
  {
    public:
      uint32_t motor_states_length;
      typedef rhino_msgs::MotorState _motor_states_type;
      _motor_states_type st_motor_states;
      _motor_states_type * motor_states;

    MotorStateList():
      motor_states_length(0), motor_states(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->motor_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motor_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motor_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motor_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_states_length);
      for( uint32_t i = 0; i < motor_states_length; i++){
      offset += this->motor_states[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t motor_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motor_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motor_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motor_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motor_states_length);
      if(motor_states_lengthT > motor_states_length)
        this->motor_states = (rhino_msgs::MotorState*)realloc(this->motor_states, motor_states_lengthT * sizeof(rhino_msgs::MotorState));
      motor_states_length = motor_states_lengthT;
      for( uint32_t i = 0; i < motor_states_length; i++){
      offset += this->st_motor_states.deserialize(inbuffer + offset);
        memcpy( &(this->motor_states[i]), &(this->st_motor_states), sizeof(rhino_msgs::MotorState));
      }
     return offset;
    }

    const char * getType(){ return "rhino_msgs/MotorStateList"; };
    const char * getMD5(){ return "9e94ccf6563ca78afce19eb097f9343c"; };

  };

}
#endif
