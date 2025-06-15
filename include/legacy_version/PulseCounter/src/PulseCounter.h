#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include <Arduino.h>
#include "driver/pcnt.h"

class PulseCounter
{
public:
    PulseCounter(int pulsePin, int16_t lowLimit = -INT16_MAX, int16_t highLimit = INT16_MAX);
    ~PulseCounter();

    bool begin();
    int16_t getCount(bool resetAfterRead = false);
    void clear();
    void end();

private:
    gpio_num_t   _pulsePin;
    int16_t      _lowLimit;
    int16_t      _highLimit;
    pcnt_unit_t  _unit;
    bool         _started;

    // automatically assign units 0…PCNT_UNIT_3
    static pcnt_unit_t  _nextUnit;
};

#endif  // PULSECOUNTER_H
