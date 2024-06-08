#include "mc_main.hpp"

void controlSlowStart()
{
    if ((!flags.initCompleteFlag) || flags.slowStartCompleteFlag || (!flags.tmrSlowStartFlag))
    {
        return;
    }
    flags.tmrSlowStartFlag = 0;
    if ((--timers.slowStartTimer) == 0)
    {
        if ((--slowStartEvents) == 0)
        {
            flags.slowStartCompleteFlag = 1;
            timers.startupTimer = TIMEBASE_STARTUP_COUNT;
            HAL_TIM_IC_Start_IT(&htim2, activeHallChannel);
        }
        else
        {
            timers.slowStartTimer = TIMEBASE_SLOW_STEP;
            commutate();
        }
    }
}

void controlStartUp()
{
    if ((!flags.slowStartCompleteFlag) || (!flags.startupInProgress) || (!flags.tmrStartupFlag))
    {
        return;
    }
    flags.tmrStartupFlag = 0;
    if ((--timers.startupTimer) == 0)
    {
        flags.startupInProgress = 0;
        if (!flags.startupCompleteFlag)
        {
            flags.stopFlag = 1;
        }
    }
}
