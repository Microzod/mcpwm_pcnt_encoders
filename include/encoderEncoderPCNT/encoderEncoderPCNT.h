#ifndef ENCODER_ENCODER_PCNT_H
#define ENCODER_ENCODER_PCNT_H

#include <Arduino.h>

#include "driver/pulse_cnt.h"


#define START_POS_DEFAULT 0
#define GLITCH_NS_DEFAULT 1000

class encoderEncoderPCNT {
  public:
    encoderEncoderPCNT(int a, int b, int enc1, int enc2, int start_pos, uint16_t glitch_ns)
	{
		
      glitch_time = glitch_ns;
      offset = start_pos;
	  pulsePinA = enc1;
	  pulsePinB = enc2;
      pin_a = a;
      pin_b = b;
      //init();
    }

    encoderEncoderPCNT(int a, int b, int enc1, int enc2, int start_pos)
	{
      offset = start_pos;
      pulsePinA = enc1;
	  pulsePinB = enc2;
      pin_a = a;
      pin_b = b;
      //init();
    }

    encoderEncoderPCNT(int a, int b, int enc1, int enc2 )
	{
      pulsePinA = enc1;
	  pulsePinB = enc2;
      pin_a = a;
      pin_b = b;
      init();
    }

    encoderEncoderPCNT(){
    }

    ~encoderEncoderPCNT(){
      deinit();
    }
    
	void init();
	//void addEncoders(int &val);
    void deinit();
    int  position();
    void setPosition(int pos);
    void zero();
    uint8_t  pin_a = 255;
    uint8_t  pin_b = 255;
	uint8_t  pulsePinA = 255;
	uint8_t  pulsePinB = 255;
    uint16_t glitch_time = GLITCH_NS_DEFAULT;
	
	int getCountA(bool resetAfterRead = false);
	int getCountB(bool resetAfterRead = false);

  private:
	
    pcnt_unit_handle_t unit;
	pcnt_unit_handle_t    _pcntA;
	pcnt_unit_handle_t    _pcntB;
    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;
	pcnt_channel_handle_t _chanA;
	pcnt_channel_handle_t _chanB;
    int16_t low_limit = INT16_MIN;
    int16_t high_limit = INT16_MAX;
    int count = 0;
    int offset = START_POS_DEFAULT;
	
	int                   _intrPriority;
	bool                  _started;
};
#endif
