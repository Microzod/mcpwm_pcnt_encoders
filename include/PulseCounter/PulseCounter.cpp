#include "PulseCounter.h"

PulseCounter::PulseCounter(int pulsePin, int16_t lowLimit, int16_t highLimit, int intrPriority)
 : _pulsePin(pulsePin)
 , _lowLimit(lowLimit)
 , _highLimit(highLimit)
 , _intrPriority(intrPriority)
 , _pcnt(nullptr)
 , _chan(nullptr)
 , _started(false)
{}

PulseCounter::~PulseCounter()
{
    end();
}

bool PulseCounter::begin()
{
	// 1) configure PCNT unit
	pcnt_unit_config_t ucfg =
	{
		.low_limit     = _lowLimit,
		.high_limit    = _highLimit,
		.intr_priority = _intrPriority,
	};
	ucfg.flags.accum_count = true;
	if (pcnt_new_unit(&ucfg, &_pcnt) != ESP_OK) return false;

	// 2) one channel, pulsePin → edge; no level gate
	pcnt_chan_config_t ccfg =
	{
		.edge_gpio_num  = (gpio_num_t)_pulsePin,
		.level_gpio_num = (gpio_num_t)-1
	};
	if (pcnt_new_channel(_pcnt, &ccfg, &_chan) != ESP_OK) return false;

	// 3) count on rising edge only
	if (pcnt_channel_set_edge_action(
			_chan,
			PCNT_CHANNEL_EDGE_ACTION_INCREASE,
			PCNT_CHANNEL_EDGE_ACTION_HOLD) != ESP_OK) return false;

	// 4) ignore level (always enabled)
	if (pcnt_channel_set_level_action(
			_chan,
			PCNT_CHANNEL_LEVEL_ACTION_KEEP,
			PCNT_CHANNEL_LEVEL_ACTION_KEEP) != ESP_OK) return false;

	// 5) clear, enable & start
	if (pcnt_unit_clear_count(_pcnt) != ESP_OK)   return false;
	if (pcnt_unit_enable(_pcnt) != ESP_OK)        return false;
	if (pcnt_unit_start(_pcnt) != ESP_OK)         return false;

	_started = true;
	return true;
}

int PulseCounter::getCount(bool resetAfterRead)
{
	if(!_started) return 0;
	
	int cnt = 0;
	
	pcnt_unit_get_count(_pcnt, &cnt);
	
	if(resetAfterRead)
		pcnt_unit_clear_count(_pcnt);
	
	return cnt;
}

void PulseCounter::clear()
{
	if (_started)
	{
		pcnt_unit_clear_count(_pcnt);
	}
}

void PulseCounter::end()
{
    if (_started)
    {
        ESP_ERROR_CHECK(pcnt_unit_disable(_pcnt));
        ESP_ERROR_CHECK(pcnt_del_channel(_chan));
        ESP_ERROR_CHECK(pcnt_del_unit(_pcnt));
        _started = false;
    }
}
