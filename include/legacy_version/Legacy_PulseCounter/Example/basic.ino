// PulseCounter::begin()
_unit = allocPcntUnit();
if (_unit == PCNT_UNIT_MAX) return false;

// RotaryEncoderPCNT::begin()
_unit = allocPcntUnit();
if (_unit == PCNT_UNIT_MAX) return false;
