#include "encoderEncoderPCNT.h"

void encoderEncoderPCNT::init(){
    
  // Unit config
  pcnt_unit_config_t unit_config = {
      .low_limit = low_limit,
      .high_limit = high_limit,
  };
  unit_config.flags.accum_count = true;

  // Create unit
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));

  // Set watch points at low and high limits to auto-accumulate overflows.
  pcnt_unit_add_watch_point(unit, low_limit);
  pcnt_unit_add_watch_point(unit, high_limit);

  // Glitch filter setup
  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = glitch_time,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));

  // Channel A setup
  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = pin_a,
      .level_gpio_num = pin_b,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_config, &chan_a));

  // Channel B setup
  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = pin_b,
      .level_gpio_num = pin_a,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_config, &chan_b));

  // Set edge and level actions for both channels
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // Enable, clear and start the PCNT unit.
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

void encoderEncoderPCNT::deinit(){
  // Free PCNT resources when destroyed.
  pcnt_unit_disable(unit);
  pcnt_del_channel(chan_a);
  pcnt_del_channel(chan_b);
  pcnt_del_unit(unit);
  
  pcnt_unit_disable(_pcntA);
  pcnt_del_channel(_chanA);
  pcnt_del_unit(_pcntA);
  
  pcnt_unit_disable(_pcntB);
  pcnt_del_channel(_chanB);
  pcnt_del_unit(_pcntB);
  
  _started = false;
}

int encoderEncoderPCNT::position(){
  pcnt_unit_get_count(unit, &count);
  return (count + offset);
}

void encoderEncoderPCNT::setPosition(int pos){
  offset = pos;
  pcnt_unit_get_count(unit, &count);
  zero();
}

void encoderEncoderPCNT::zero(){
  pcnt_unit_clear_count(unit);
}

void encoderEncoderPCNT::addEncoders(int pinA, int pinB)
{
	_pulsePinA = pinA;
	_pulsePinB = pinB;
	
	// Unit config
	pcnt_unit_config_t ucfg = {
		.low_limit = low_limit,
		.high_limit = high_limit,
	};
	unit_config.flags.accum_count = true;
	
	// Create unit
	ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &_pcntA));
	ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &_pcntB));
	
	// Glitch filter setup
	pcnt_glitch_filter_config_t filter_cfg = {
		.max_glitch_ns = glitch_time,
	};
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(_pcntA, &filter_cfg));
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(_pcntB, &filter_cfg));
	
	// Channel A setup
	pcnt_chan_config_t chan_a_cfg = {
		.edge_gpio_num = pin_a,
		.level_gpio_num = (gpio_num_t)-1,
	};
	ESP_ERROR_CHECK(pcnt_new_channel(_pcntA, &chan_a_cfg, &_chanA));
	
	// Channel B setup
	pcnt_chan_config_t chan_b_cfg = {
		.edge_gpio_num = pin_b,
		.level_gpio_num = (gpio_num_t)-1,
	};
	ESP_ERROR_CHECK(pcnt_new_channel(_pcntB, &chan_b_cfg, &_chanB));
	
	// Set edge and level actions for both channels
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(_chanA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(_chanA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(_chanB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(_chanB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
	
	// Enable, clear and start the PCNT unit.
	ESP_ERROR_CHECK(pcnt_unit_enable(_pcntA));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(_pcntA));
	ESP_ERROR_CHECK(pcnt_unit_start(_pcntA));
	
	ESP_ERROR_CHECK(pcnt_unit_enable(_pcntB));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(_pcntB));
	ESP_ERROR_CHECK(pcnt_unit_start(_pcntB));
	
	_started = true;
}

int encoderEncoderPCNT::getCountA(bool resetAfterRead)
{
	if(!_started) return 0;
	
	int cnt = 0;
	
	pcnt_unit_get_count(_pcntA, &cnt);
	
	if(resetAfterRead)
		pcnt_unit_clear_count(_pcntA);
	
	return cnt;
}

int encoderEncoderPCNT::getCountB(bool resetAfterRead)
{
	if(!_started) return 0;
	
	int cnt = 0;
	
	pcnt_unit_get_count(_pcntB, &cnt);
	
	if(resetAfterRead)
		pcnt_unit_clear_count(_pcntB);
	
	return cnt;
}
