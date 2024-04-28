#include "application.hpp"

void controlSlowStart()
{
    if (!(flags.initCompleteFlag))
    {
        return;
    }
    if (flags.slowStartCompleteFlag)
    {
        return;
    }
    if (!(flags.tmrSlowStartFlag))
    {
        return;
    }
    flags.tmrSlowStartFlag = 0;
    if (--tmrSlowStartTimer == 0)
    {
        if (--slowStartEvents == 0)
        {
            flags.slowStartCompleteFlag = 1;
            tmrStartupTimer = TIMEBASE_STARTUP_COUNT;
            switch (settings.sensorType)
            {
            case SensorType_SENSORLESS_1:
                __HAL_TIM_SET_COUNTER(&htim2, tmr2CommTime);
                isrState = IsrState_COMMUTATE;
                HAL_TIM_Base_Start_IT(&htim2);
                break;
            case SensorType_SENSORED:
                switch (gpioPin)
                {
                case M1_BEMF_A_Pin:
                    HAL_TIM_IC_Start_IT(&htim2, M1_HALL_A);
                    break;
                case M1_BEMF_B_Pin:
                    HAL_TIM_IC_Start_IT(&htim2, M1_HALL_B);
                    break;
                case M1_BEMF_C_Pin:
                    HAL_TIM_IC_Start_IT(&htim2, M1_HALL_C);
                    break;
                default:
                    break;
                }
                break;
            }
        }
        else
        {
            tmrSlowStartTimer = TIMEBASE_SLOW_STEP;
            commutate();
        }
    }
}

void controlStartUp()
{
    if (!(flags.slowStartCompleteFlag))
    {
        return;
    }
    if (!(flags.startupInProgress))
    {
        return;
    }
    if (!(flags.tmrStartupFlag))
    {
        return;
    }
    flags.tmrStartupFlag = 0;
    if (--tmrStartupTimer == 0)
    {
        flags.startupInProgress = 0;
        if (!(flags.startupCompleteFlag))
        {
            flags.stopFlag = 1;
        }
    }
}