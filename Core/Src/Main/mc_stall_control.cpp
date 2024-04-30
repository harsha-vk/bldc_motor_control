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
    if (--timers.stallTimer == 0)
    {
        flags.stopFlag = 1;
    }
}
