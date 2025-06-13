#ifndef ENCODER_ENCODER_PCNT_H
#define ENCODER_ENCODER_PCNT_H

#include "driver/pulse_cnt.h"

#define START_POS_DEFAULT 0
#define GLITCH_NS_DEFAULT 1000

class encoderEncoderPCNT {
  public:
    encoderEncoderPCNT(int a, int b, int start_pos, uint16_t glitch_ns){
      glitch_time = glitch_ns;
      offset = start_pos;
      pin_a = a;
      pin_b = b;
      init();
    }

    encoderEncoderPCNT(int a, int b, int start_pos){
      offset = start_pos;
      pin_a = a;
      pin_b = b;
      init();
    }

    encoderEncoderPCNT(int a, int b){
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
    void deinit();
    int  position();
    void setPosition(int pos);
    void zero();
    uint8_t  pin_a = 255;
    uint8_t  pin_b = 255;
    uint16_t glitch_time = GLITCH_NS_DEFAULT;
	void addEncoders(int pinA, int pinB);
	int getCountA(bool resetAfterRead = false);
	int getCountB(bool resetAfterRead = false);

  private:
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;
    int16_t low_limit = INT16_MIN;
    int16_t high_limit = INT16_MAX;
    int count = 0;
    int offset = START_POS_DEFAULT;
	int            		  _pulsePinA;
	int            		  _pulsePinB;
	int16_t               _lowLimit;
	int16_t               _highLimit;
	int                   _intrPriority;
	pcnt_unit_handle_t    _pcntA;
	pcnt_unit_handle_t    _pcntB;
	pcnt_channel_handle_t _chanA;
	pcnt_channel_handle_t _chanB;
	bool                  _started;
};
#endif
