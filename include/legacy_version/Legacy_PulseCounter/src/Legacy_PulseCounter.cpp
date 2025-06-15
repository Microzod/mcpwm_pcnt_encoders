#include "Legacy_PulseCounter.h"

pcnt_unit_t Legacy_PulseCounter::_nextUnit = PCNT_UNIT_0;

Legacy_PulseCounter::Legacy_PulseCounter
(
    int       pulsePin,
    int16_t   lowLimit,
    int16_t   highLimit
)
{
    _pulsePin  = (gpio_num_t)pulsePin;
    _lowLimit  = lowLimit;
    _highLimit = highLimit;
    _started   = false;
    _unit      = PCNT_UNIT_MAX;
}

Legacy_PulseCounter::~Legacy_PulseCounter()
{
    end();
}

bool Legacy_PulseCounter::begin()
{
    if (_nextUnit >= PCNT_UNIT_MAX) { return false; }// no more hardware units left

    _unit = allocPcntUnit();
    if (_unit == PCNT_UNIT_MAX) return false;

    pcnt_config_t config = {};
    config.pulse_gpio_num = _pulsePin;
    config.ctrl_gpio_num  = (gpio_num_t)-1;    // no gating
    config.unit           = _unit;
    config.channel        = PCNT_CHANNEL_0;
    config.pos_mode       = PCNT_COUNT_INC;    // ↑ → +1
    config.neg_mode       = PCNT_COUNT_DIS;    // ↓ → no change
    config.lctrl_mode     = PCNT_MODE_KEEP;    // always keep counting
    config.hctrl_mode     = PCNT_MODE_KEEP;
    config.counter_h_lim  = _highLimit;
    config.counter_l_lim  = _lowLimit;

    if (pcnt_unit_config(_unit, &config) != ESP_OK){ return false; }

    if (pcnt_counter_clear(_unit) != ESP_OK || pcnt_counter_resume(_unit) != ESP_OK) { return false; }

    _started = true;
    return true;
}

int16_t Legacy_PulseCounter::getCount( bool resetAfterRead )
{
    if (!_started) { return 0; }

    int16_t cnt = 0;
    pcnt_get_counter_value(_unit, &cnt);

    if (resetAfterRead)
    {
        pcnt_counter_clear(_unit);
    }

    return cnt;
}

void PulseCounter::clear()
{
    if (_started)
    {
        pcnt_counter_clear(_unit);
    }
}

void PulseCounter::end()
{
    if (_started)
    {
        pcnt_counter_pause(_unit);
        pcnt_unit_disable(_unit);
        _started = false;
    }
}
