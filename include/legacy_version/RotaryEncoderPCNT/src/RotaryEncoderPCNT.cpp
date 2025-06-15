#include "RotaryEncoderPCNT.h"

pcnt_unit_t RotaryEncoderPCNT::_nextUnit = PCNT_UNIT_0;

RotaryEncoderPCNT::RotaryEncoderPCNT
(
    int       pinA,
    int       pinB,
    int16_t   lowLimit,
    int16_t   highLimit
)
{
    _pinA      = (gpio_num_t)pinA;
    _pinB      = (gpio_num_t)pinB;
    _lowLimit  = lowLimit;
    _highLimit = highLimit;
    _unit      = PCNT_UNIT_MAX;
    _started   = false;
    _offset    = 0;
}

RotaryEncoderPCNT::~RotaryEncoderPCNT()
{
    deinit();
}

bool RotaryEncoderPCNT::begin()
{
    if (_nextUnit >= PCNT_UNIT_MAX)
    {
        return false;
    }

    _unit = _nextUnit++;

    // ── channel 0: A rising/falling vs. B level ──
    pcnt_config_t cfgA = {};
    cfgA.pulse_gpio_num = _pinA;
    cfgA.ctrl_gpio_num  = _pinB;
    cfgA.unit           = _unit;
    cfgA.channel        = PCNT_CHANNEL_0;
    cfgA.pos_mode       = PCNT_COUNT_INC;    // A↑ → +1
    cfgA.neg_mode       = PCNT_COUNT_DEC;    // A↓ → -1
    cfgA.lctrl_mode     = PCNT_MODE_KEEP;    // B low → keep counting direction
    cfgA.hctrl_mode     = PCNT_MODE_REVERSE; // B high → reverse edge sense
    cfgA.counter_h_lim  = _highLimit;
    cfgA.counter_l_lim  = _lowLimit;

    if (pcnt_unit_config(_unit, &cfgA) != ESP_OK)
    {
        return false;
    }

    // ── channel 1: B rising/falling vs. A level ──
    pcnt_config_t cfgB = {};
    cfgB.pulse_gpio_num = _pinB;
    cfgB.ctrl_gpio_num  = _pinA;
    cfgB.unit           = _unit;
    cfgB.channel        = PCNT_CHANNEL_1;
    cfgB.pos_mode       = PCNT_COUNT_DEC;    // B↑ → -1
    cfgB.neg_mode       = PCNT_COUNT_INC;    // B↓ → +1
    cfgB.lctrl_mode     = PCNT_MODE_KEEP;
    cfgB.hctrl_mode     = PCNT_MODE_REVERSE;
    cfgB.counter_h_lim  = _highLimit;
    cfgB.counter_l_lim  = _lowLimit;

    if (pcnt_unit_config(_unit, &cfgB) != ESP_OK)
    {
        return false;
    }

    if (pcnt_counter_clear(_unit) != ESP_OK ||
        pcnt_counter_resume(_unit) != ESP_OK)
    {
        return false;
    }

    _started = true;
    return true;
}

void RotaryEncoderPCNT::deinit()
{
    if (_started)
    {
        pcnt_counter_pause(_unit);
        pcnt_unit_disable(_unit);
        _started = false;
    }
}

int32_t RotaryEncoderPCNT::getPosition()
{
    if (!_started)
    {
        return 0;
    }

    int16_t cnt = 0;
    pcnt_get_counter_value(_unit, &cnt);
    return (int32_t)cnt + _offset;
}

void RotaryEncoderPCNT::setPosition
(
    int32_t pos
)
{
    _offset = pos;
    zero();
}

void RotaryEncoderPCNT::zero()
{
    if (_started)
    {
        pcnt_counter_clear(_unit);
    }
}
