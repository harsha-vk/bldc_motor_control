#include "application.hpp"

void stallControl()
{
    if (!(flags.initCompleteFlag))
    {
        return;
    }
    if (!(flags.startupCompleteFlag))
    {
        return;
    }
    if (!(flags.tmrStallFlag))
    {
        return;
    }
    flags.tmrStallFlag = 0;
    if (--tmrStallTimer == 0)
    {
        flags.stopFlag = 1;
    }
    if (--tmrStallCheckTimer == 0)
    {
        tmrStallCheckTimer = TIMEBASE_STALLCHECK_COUNT;
        if (tmr2CommTime > MAX_TMR2_PRESET)
        {
            flags.stopFlag = 1;
        }
    }
}
