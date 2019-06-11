#ifndef _ROS_SERVICE_genImg_h
#define _ROS_SERVICE_genImg_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace storm
{

static const char GENIMG[] = "storm/genImg";

  class genImgRequest : public ros::Msg
  {
    public:
      typedef int32_t _nothing_type;
      _nothing_type nothing;

    genImgRequest():
      nothing(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_nothing;
      u_nothing.real = this->nothing;
      *(outbuffer + offset + 0) = (u_nothing.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nothing.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nothing.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nothing.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nothing);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_nothing;
      u_nothing.base = 0;
      u_nothing.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nothing.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nothing.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nothing.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nothing = u_nothing.real;
      offset += sizeof(this->nothing);
     return offset;
    }

    const char * getType(){ return GENIMG; };
    const char * getMD5(){ return "30d963ed1a32cf29371872f3a935865b"; };

  };

  class genImgResponse : public ros::Msg
  {
    public:
      typedef const char* _fname_type;
      _fname_type fname;

    genImgResponse():
      fname("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_fname = strlen(this->fname);
      varToArr(outbuffer + offset, length_fname);
      offset += 4;
      memcpy(outbuffer + offset, this->fname, length_fname);
      offset += length_fname;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_fname;
      arrToVar(length_fname, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_fname; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_fname-1]=0;
      this->fname = (char *)(inbuffer + offset-1);
      offset += length_fname;
     return offset;
    }

    const char * getType(){ return GENIMG; };
    const char * getMD5(){ return "91162659a2113bc43b39b33479a0b6e0"; };

  };

  class genImg {
    public:
    typedef genImgRequest Request;
    typedef genImgResponse Response;
  };

}
#endif
