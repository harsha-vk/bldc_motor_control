#include "application.hpp"
#include "mc_main.hpp"

MC_Timers_t timers;
MC_Flags_t flags;
uint8_t slowStartEvents;
uint32_t ADC_BUFFER_ARRAY[ADC_BUFFER_LENGTH];
uint8_t stepNumber;
MC::PIDController *pidController = new MC::PIDController(0, 0, 0, 0, 0);
McDataMsg mcData = McDataMsg_init_zero;
McParamsMsg mcParams = McParamsMsg_init_zero;

uint32_t timeBaseCount;

void serialToData();

void setup()
{
    flags.powerFlag = 0;
    flags.stopFlag = 1;
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
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
    pidManager();
    serialToData();
    dataToSerial();
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
        flags.tmrPidFlag = 1;
        flags.tmrSerialReadFlag = 1;
        flags.tmrSerialWriteFlag = 1;
    }
}

void warmUpControl()
{
    if (flags.warmupCompleteFlag || (!flags.tmrWarmupFlag))
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
