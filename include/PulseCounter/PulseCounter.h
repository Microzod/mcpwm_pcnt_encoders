#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include <Arduino.h>
#include "driver/pulse_cnt.h"

class PulseCounter
{
public:
	/// ctor: specify the pulse input pin; you can optionally tune your count limits
	PulseCounter(int pulsePin, int16_t lowLimit = -INT16_MAX, int16_t highLimit = INT16_MAX, int intrPriority = 1);
	~PulseCounter();
	/// initialize the PCNT unit & channel; returns true on success
	bool begin();
	
	/// read & clear the accumulated count (pulses since last clear)
	int getCount(bool resetAfterRead = false);
	
	/// just clear the count without reading
	void clear();
	
	void end();
	
private:
	int            		  _pulsePin;
	int16_t               _lowLimit, _highLimit;
	int                   _intrPriority;
	pcnt_unit_handle_t    _pcnt;
	pcnt_channel_handle_t _chan;
	bool                  _started;
};

#endif  // PULSECOUNTER_H
