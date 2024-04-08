#include "application.hpp"

MC_IsrState_e isrState;
uint16_t tempCommTime;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (TIM2 == htim->Instance)
    {
        if (IsrState_ZERO_DETECT == isrState)
        {
            __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3);

            zc = __HAL_TIM_GET_COUNTER(&htim2);

            if (flags.startupCompleteFlag)
            {
                HAL_TIM_Base_Stop(&htim2);
                __HAL_TIM_SET_COUNTER(&htim2, commAfterZc);
                HAL_TIM_Base_Start(&htim2);
            }

            isrState = IsrState_COMMUTATE;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (TIM2 == htim->Instance)
    {
        switch (isrState)
        {
        case IsrState_ZERO_DETECT:
            if (flags.startupCompleteFlag)
            {
                tmr2CommTime += (expectedZc >> 2);
            }
        case IsrState_COMMUTATE:
            commutate();
            if (flags.risingBemfFlag)
            {
                tempCommTime = tmr2CommTime + BLANKING_COUNT;
                while (__HAL_TIM_GET_COUNTER(&htim2) < BLANKING_COUNT);
                HAL_TIM_Base_Stop(&htim2);
                __HAL_TIM_SET_COUNTER(&htim2, tempCommTime + 1);
                HAL_TIM_Base_Start(&htim2);

                // while (HAL_GPIO_ReadPin((GPIO_TypeDef *)Bit32, Bit16) != GPIO_PIN_RESET)
                // {
                //      if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) break;
                // }

                if (!(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)))
                {
                    switch (commState)
                    {
                    case 1:
                        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
                        break;
                    case 2:
                        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
                        break;
                    case 3:
                        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
                        break;
                    case 4:
                        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
                        break;
                    case 5:
                        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
                        break;
                    case 6:
                        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
                        break;
                    default:
                        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
                        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
                        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);
                        break;
                    }
                    isrState = IsrState_ZERO_DETECT;
                }
            }
            else
            {
                expectedZc = (tmr2CommTime >> 1) | 0x8000; // CT(n)/2
                zcError = zc - expectedZc; // ZCE(n) = ZC(n)-(CT(n)/2)
                temp = zcError;
                if (temp & 0x8000)
                {
                    temp = ~temp + 1; // absolute value
                }
                // stop forced commutation if zero cross detected within middle half of comm period
                if (temp < ((uint16_t)-expectedZc >> 1))
                {
                    flags.startupCompleteFlag = 1;
                    tmrStallTimer = TIMEBASE_STALL_COUNT;
                }
                tmr2CommTime -= (zcError >> ERROR_SCALE); // -CT(n+1) = -CT(n) - ZCE(n)*Error_Gain
                tempCommTime = tmr2CommTime + FIXED_ADVANCE_COUNT;
                HAL_TIM_Base_Stop(&htim2);
                __HAL_TIM_SET_COUNTER(&htim2, tempCommTime + 1);
                HAL_TIM_Base_Start(&htim2);
                isrState = IsrState_COMMUTATE;
                commAfterZc = expectedZc + ADVANCE_COUNT;
            }
            break;
        default:
            flags.stopFlag = 1;
            break;
        }
    }
}
