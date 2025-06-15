#ifndef LEGACY_PULSECOUNTER_H
#define LEGACYPULSECOUNTER_H

#include <Arduino.h>
#include "driver/pcnt.h"
#include "PCNTAllocator.h"

class Legacy_PulseCounter
{
public:
    Legacy_PulseCounter(int pulsePin, int16_t lowLimit = -INT16_MAX, int16_t highLimit = INT16_MAX);
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

#endif  // LEGACY_PULSECOUNTER_H
