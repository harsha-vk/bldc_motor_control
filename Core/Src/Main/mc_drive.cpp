#include "mc_main.hpp"

const uint32_t maxOutputPulse = MAX_PWM_PULSE;
uint32_t activeHallChannel;

void tmr1PwmStart(uint32_t channel);
void tmr1PwmNStart(uint32_t channel);
void tmr1PwmStopPwmNStop(uint32_t channel);
void tmr1SetCompare(uint32_t channel, uint32_t compare);
void tmr2SetCapturePolarity(uint32_t channel, uint32_t polarity);

void commutate()
{
    switch (stepNumber)
    {
    case 1:
        tmr1PwmStopPwmNStop(M1_PWM_C_H);
        tmr1PwmStart(M1_PWM_A_H);
        tmr1PwmNStart(M1_PWM_B_L);
        activeHallChannel = M1_HALL_C;
        break;
    case 2:
        tmr1PwmStopPwmNStop(M1_PWM_B_L);
        tmr1PwmStart(M1_PWM_A_H);
        tmr1PwmNStart(M1_PWM_C_L);
        activeHallChannel = M1_HALL_B;
        break;
    case 3:
        tmr1PwmStopPwmNStop(M1_PWM_A_H);
        tmr1PwmStart(M1_PWM_B_H);
        tmr1PwmNStart(M1_PWM_C_L);
        activeHallChannel = M1_HALL_A;
        break;
    case 4:
        tmr1PwmStopPwmNStop(M1_PWM_C_L);
        tmr1PwmStart(M1_PWM_B_H);
        tmr1PwmNStart(M1_PWM_A_L);
        activeHallChannel = M1_HALL_C;
        break;
    case 5:
        tmr1PwmStopPwmNStop(M1_PWM_B_H);
        tmr1PwmStart(M1_PWM_C_H);
        tmr1PwmNStart(M1_PWM_A_L);
        activeHallChannel = M1_HALL_B;
        break;
    case 6:
        tmr1PwmStopPwmNStop(M1_PWM_A_L);
        tmr1PwmStart(M1_PWM_C_H);
        tmr1PwmNStart(M1_PWM_B_L);
        activeHallChannel = M1_HALL_A;
        break;
    default:
        tmr1PwmStopPwmNStop(TIM_CHANNEL_1);
        tmr1PwmStopPwmNStop(TIM_CHANNEL_2);
        tmr1PwmStopPwmNStop(TIM_CHANNEL_3);
        break;
    }

    uint8_t isCCW = (Rotate_CCW == readings->direction);
    switch (stepNumber)
    {
    case 1:
        stepNumber = isCCW ? 7 : stepNumber;
    case 3:
    case 5:
        tmr2SetCapturePolarity(activeHallChannel, isCCW ? HALL_RISING : HALL_FALLING);
        break;
    case 6:
        stepNumber = (!isCCW) ? 0 : stepNumber;
    case 4:
    case 2:
        tmr2SetCapturePolarity(activeHallChannel, isCCW ? HALL_FALLING : HALL_RISING);
        break;
    default:
        break;
    }
    stepNumber = stepNumber + readings->direction;
}

void tmr1PwmStart(uint32_t channel)
{
    switch (settings.modulationType)
    {
    case ModulationType_HIGH_SIDE:
        tmr1SetCompare(channel, readings->outputPulse);
        break;
    case ModulationType_LOW_SIDE:
        tmr1SetCompare(channel, maxOutputPulse);
        break;
    }
    HAL_TIM_PWM_Start(&htim1, channel);
}

void tmr1PwmNStart(uint32_t channel)
{
    switch (settings.modulationType)
    {
    case ModulationType_HIGH_SIDE:
        tmr1SetCompare(channel, maxOutputPulse);
        break;
    case ModulationType_LOW_SIDE:
        tmr1SetCompare(channel, readings->outputPulse);
        break;
    }
    HAL_TIMEx_PWMN_Start(&htim1, channel);
}

void tmr1PwmStopPwmNStop(uint32_t channel)
{
    HAL_TIM_PWM_Stop(&htim1, channel);
    HAL_TIMEx_PWMN_Stop(&htim1, channel);
}

void tmr1SetCompare(uint32_t channel, uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(&htim1, channel, compare);
}

void tmr2SetCapturePolarity(uint32_t channel, uint32_t polarity)
{
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, channel, polarity);
}
