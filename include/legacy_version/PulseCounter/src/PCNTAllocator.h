#ifndef _PCNT_ALLOCATOR_H_
#define _PCNT_ALLOCATOR_H_

#include "driver/pcnt.h"

static pcnt_unit_t __nextPcntUnit = PCNT_UNIT_0;

inline pcnt_unit_t allocPcntUnit()
{
    if (__nextPcntUnit >= PCNT_UNIT_MAX) {
        return PCNT_UNIT_MAX;  // out of units
    }
    return __nextPcntUnit++;
}

#endif
