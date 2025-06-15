#ifndef LEGACY_ROTARYENCODERPCNT_H
#define LEGACY_ROTARYENCODERPCNT_H

#include <Arduino.h>
#include "driver/pcnt.h"
#include "PCNTAllocator.h"

class Legacy_RotaryEncoderPCNT
{
public:
    Legacy_RotaryEncoderPCNT(int pinA, int pinB,
                      int16_t lowLimit = -INT16_MAX,
                      int16_t highLimit = INT16_MAX);
    ~Legacy_RotaryEncoderPCNT();

    bool      begin();
    void      deinit();
    int32_t   getPosition();
    void      setPosition(int32_t pos);
    void      zero();

private:
    gpio_num_t   _pinA;
    gpio_num_t   _pinB;
    int16_t      _lowLimit;
    int16_t      _highLimit;
    pcnt_unit_t  _unit;
    bool         _started;
    int32_t      _offset;

    // assign units 0…PCNT_UNIT_3
    static pcnt_unit_t  _nextUnit;
};

#endif  // ROTARYENCODERPCNT_H
