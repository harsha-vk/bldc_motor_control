#include "application.hpp"

MC_Settings_t settings;
MC_Timers_t timers;
MC_Flags_t flags;
uint8_t slowStartEvents;
uint32_t ADC_BUFFER_ARRAY[ADC_BUFFER_LENGTH];
uint32_t dutyCycle;
uint8_t commState;

uint32_t timeBaseCount;

void setup()
{
    flags.stopFlag = 1;
}

void loop()
{
    if (flags.stopFlag)
    {
        initSystem();
    }
    timeBaseManager();
    warmUpControl();
    controlSlowStart();
    controlStartUp();
    stallControl();
    speedManager();
}

void timeBaseManager()
{
    if ((timeBaseCount - HAL_GetTick()) >= TIMEBASE_10ms)
    {
        timeBaseCount = HAL_GetTick();
        flags.tmrWarmupFlag = 1;
        flags.tmrSlowStartFlag = 1;
        flags.tmrStartupFlag = 1;
        flags.tmrStallFlag = 1;
        flags.tmrDutyFlag = 1;
    }
}

void warmUpControl()
{
    if (flags.warmupCompleteFlag)
    {
        return;
    }
    if (!(flags.tmrWarmupFlag))
    {
        return;
    }
    flags.tmrWarmupFlag = 0;
    if (timers.warmupTimer)
    {
        timers.warmupTimer--;
    }
    else
    {
        if (flags.runFlag)
        {
            flags.warmupCompleteFlag = 1;
            initDriver();
        }
    }
}
