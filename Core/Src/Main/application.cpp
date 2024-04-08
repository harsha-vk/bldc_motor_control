#include "application.hpp"

uint8_t commState;

uint16_t tmr2CommTime;
uint16_t zc;
uint16_t expectedZc;
uint16_t commAfterZc;

uint8_t slowStartEvents;
uint8_t tmrSlowStartTimer;
uint8_t tmrStartupTimer;
uint8_t tmrStallTimer;
uint8_t tmrWarmupTimer;
uint8_t tmrDutyTimer;
uint8_t tmrStallCheckTimer;
uint32_t timeBaseCount;

MC_Flags_t flags;

int zcError;
int temp;
int8_t ctemp;
uint8_t rampedSpeed;

uint32_t dutyCycle;
const uint32_t maxDutyCycle = MAX_DUTY_CYCLE;

MC_Settings_t settings;

uint32_t adcVal[8];

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
    if (tmrWarmupTimer)
    {
        tmrWarmupTimer--;
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
