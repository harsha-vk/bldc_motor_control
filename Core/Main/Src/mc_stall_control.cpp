#include "mc_main.hpp"

void stallControl()
{
    if ((!flags.startupCompleteFlag) || (!flags.tmrStallFlag))
    {
        return;
    }
    flags.tmrStallFlag = 0;
    if ((--timers.stallTimer) == 0)
    {
        flags.stopFlag = 1;
    }
}
