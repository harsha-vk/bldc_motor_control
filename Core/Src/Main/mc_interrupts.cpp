#include "application.hpp"

uint32_t tmr2CommCnt;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (TIM2 == htim->Instance)
    {
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC2);
        HAL_TIM_Base_Stop(&htim2);

        commutate();

        tmr2CommCnt = __HAL_TIM_GET_COUNTER(&htim2);
        
        flags.startupCompleteFlag = 1;
        timers.stallTimer = TIMEBASE_STALL_COUNT;
        
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_IC_Start_IT(&htim2, activeHallChannel);
    }
}
