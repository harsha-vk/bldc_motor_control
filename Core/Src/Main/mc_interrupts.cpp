#include "mc_main.hpp"

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

        readings->setSpeedFdbk((uint16_t)__HAL_TIM_GET_COUNTER(&htim2));

        flags.startupCompleteFlag = 1;
        timers.stallTimer = TIMEBASE_STALL_COUNT;

        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_IC_Start_IT(&htim2, activeHallChannel);
    }
}
