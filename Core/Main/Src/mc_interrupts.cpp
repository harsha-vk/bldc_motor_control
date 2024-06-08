#include "mc_main.hpp"

void filteredSpeedFdbk(int16_t cntVal);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((!flags.stopFlag) && (USR_BTN_Pin == GPIO_Pin))
    {
        if (!flags.powerFlag)
        {
            flags.powerFlag = 1;
            HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            flags.powerFlag = 0;
            HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (TIM2 == htim->Instance)
    {
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC2);
        HAL_TIM_Base_Stop(&htim2);

        commutate();

        filteredSpeedFdbk((int16_t)__HAL_TIM_GET_COUNTER(&htim2));

        flags.startupCompleteFlag = 1;
        timers.stallTimer = TIMEBASE_STALL_COUNT;

        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_IC_Start_IT(&htim2, activeHallChannel);
    }
}

// TODO: ALPHA from rxData settings??
#define ALPHA 9
void filteredSpeedFdbk(int16_t cntVal)
{
    // Electrical frequency in Hz
    int16_t elFreqHz = TMR2_COUNTS_PER_SEC / (cntVal * 6);
    // Equivalent mechanical speed in RPM
    int16_t mechSpeedRpm = elFreqHz * 60 / POLE_PAIRS;
    // Weighted moving average
    txData.speed_fdbk = (txData.speed_fdbk * ALPHA / 100) + (mechSpeedRpm * (100 - ALPHA) / 100);
}
