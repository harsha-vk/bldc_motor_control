#include "application.hpp"

typedef enum : uint8_t
{
    Pwm_STOP,
    Pwm_START
} Pwm_e;

const uint32_t maxDutyCycle = MAX_DUTY_CYCLE;
uint32_t activeHallChannel;

void tmr1Pwm(uint32_t channel, Pwm_e state);
void tmr1PwmN(uint32_t channel, Pwm_e state);
void tmr1SetCompare(uint32_t channel, uint32_t compare);
void tmr2SetCapturePolarity(uint32_t channel, uint32_t polarity);

// TODO: commutate for CW and CCW
void commutate()
{
    switch (commState)
    {
    case 1:
        tmr1Pwm(M1_PWM_C_H, Pwm_STOP);
        tmr1Pwm(M1_PWM_A_H, Pwm_START);
        tmr1PwmN(M1_PWM_B_L, Pwm_START);
        activeHallChannel = M1_HALL_C;
        tmr2SetCapturePolarity(M1_HALL_C, HALL_FALLING);
        break;
    case 2:
        tmr1PwmN(M1_PWM_B_L, Pwm_STOP);
        // tmr1Pwm(M1_PWM_A_H, Pwm_START);
        tmr1PwmN(M1_PWM_C_L, Pwm_START);
        activeHallChannel = M1_HALL_B;
        tmr2SetCapturePolarity(M1_HALL_B, HALL_RISING);
        break;
    case 3:
        tmr1Pwm(M1_PWM_A_H, Pwm_STOP);
        tmr1Pwm(M1_PWM_B_H, Pwm_START);
        // tmr1PwmN(M1_PWM_C_L, Pwm_START);
        activeHallChannel = M1_HALL_A;
        tmr2SetCapturePolarity(M1_HALL_A, HALL_FALLING);
        break;
    case 4:
        tmr1PwmN(M1_PWM_C_L, Pwm_STOP);
        // tmr1Pwm(M1_PWM_B_H, Pwm_START);
        tmr1PwmN(M1_PWM_A_L, Pwm_START);
        activeHallChannel = M1_HALL_C;
        tmr2SetCapturePolarity(M1_HALL_C, HALL_RISING);
        break;
    case 5:
        tmr1Pwm(M1_PWM_B_H, Pwm_STOP);
        tmr1Pwm(M1_PWM_C_H, Pwm_START);
        // tmr1PwmN(M1_PWM_A_L, Pwm_START);
        activeHallChannel = M1_HALL_B;
        tmr2SetCapturePolarity(M1_HALL_B, HALL_FALLING);
        break;
    case 6:
        tmr1PwmN(M1_PWM_A_L, Pwm_STOP);
        // tmr1Pwm(M1_PWM_C_H, Pwm_START);
        tmr1PwmN(M1_PWM_B_L, Pwm_START);
        activeHallChannel = M1_HALL_A;
        tmr2SetCapturePolarity(M1_HALL_A, HALL_RISING);
        commState = 0;
        break;
    default:
        tmr1Pwm(TIM_CHANNEL_ALL, Pwm_STOP);
        tmr1PwmN(TIM_CHANNEL_ALL, Pwm_STOP);
        commState = 0;
        break;
    }
    commState++;
}

void tmr1Pwm(uint32_t channel, Pwm_e state)
{
    switch (state)
    {
    case Pwm_START:
        switch (settings.modulationType)
        {
        case ModulationType_HIGH_SIDE:
            tmr1SetCompare(channel, dutyCycle);
            break;
        case ModulationType_LOW_SIDE:
            tmr1SetCompare(channel, maxDutyCycle);
            break;
        }
        HAL_TIM_PWM_Start(&htim1, channel);
        break;
    case Pwm_STOP:
        HAL_TIM_PWM_Stop(&htim1, channel);
        break;
    }
}

void tmr1PwmN(uint32_t channel, Pwm_e state)
{
    switch (state)
    {
    case Pwm_START:
        switch (settings.modulationType)
        {
        case ModulationType_HIGH_SIDE:
            tmr1SetCompare(channel, maxDutyCycle);
            break;
        case ModulationType_LOW_SIDE:
            tmr1SetCompare(channel, dutyCycle);
            break;
        }
        HAL_TIMEx_PWMN_Start(&htim1, channel);
        break;
    case Pwm_STOP:
        HAL_TIMEx_PWMN_Stop(&htim1, channel);
        break;
    }
}

void tmr1SetCompare(uint32_t channel, uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(&htim1, channel, compare);
}

void tmr2SetCapturePolarity(uint32_t channel, uint32_t polarity)
{
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, channel, polarity);
}
