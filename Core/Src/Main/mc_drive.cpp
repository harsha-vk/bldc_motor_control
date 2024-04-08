#include "application.hpp"

typedef enum : uint8_t
{
    Pwm_STOP,
    Pwm_START
} Pwm_e;

void tmr1Pwm(uint32_t channel, Pwm_e state);
void tmr1PwmN(uint32_t channel, Pwm_e state);
void tmr1SetCompare(uint32_t channel, uint32_t compare);
void tmr2SetCapturePolarity(uint32_t channel, uint32_t polarity);

void commutate()
{
    switch (settings.motorControlMode)
    {
    case MotorControlMode_6STEP:
        switch (commState)
        {
        case 1:
            tmr1Pwm(TIM_CHANNEL_3, Pwm_STOP);
            tmr1Pwm(TIM_CHANNEL_1, Pwm_START);
            tmr1PwmN(TIM_CHANNEL_2, Pwm_START);
            tmr2SetCapturePolarity(TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
            flags.risingBemfFlag = 0;
            break;
        case 2:
            tmr1PwmN(TIM_CHANNEL_2, Pwm_STOP);
            // tmr1Pwm(TIM_CHANNEL_1, Pwm_START);
            tmr1PwmN(TIM_CHANNEL_3, Pwm_START);
            tmr2SetCapturePolarity(TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            flags.risingBemfFlag = 1;
            break;
        case 3:
            tmr1Pwm(TIM_CHANNEL_1, Pwm_STOP);
            tmr1Pwm(TIM_CHANNEL_2, Pwm_START);
            // tmr1PwmN(TIM_CHANNEL_3, Pwm_START);
            tmr2SetCapturePolarity(TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            flags.risingBemfFlag = 0;
            break;
        case 4:
            tmr1PwmN(TIM_CHANNEL_3, Pwm_STOP);
            // tmr1Pwm(TIM_CHANNEL_2, Pwm_START);
            tmr1PwmN(TIM_CHANNEL_1, Pwm_START);
            tmr2SetCapturePolarity(TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
            flags.risingBemfFlag = 1;
            break;
        case 5:
            tmr1Pwm(TIM_CHANNEL_2, Pwm_STOP);
            tmr1Pwm(TIM_CHANNEL_3, Pwm_START);
            // tmr1PwmN(TIM_CHANNEL_1, Pwm_START);
            tmr2SetCapturePolarity(TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
            flags.risingBemfFlag = 0;
            break;
        case 6:
            tmr1PwmN(TIM_CHANNEL_1, Pwm_STOP);
            // tmr1Pwm(TIM_CHANNEL_3, Pwm_START);
            tmr1PwmN(TIM_CHANNEL_2, Pwm_START);
            tmr2SetCapturePolarity(TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            flags.risingBemfFlag = 1;
            commState = 0;
            break;
        default:
            tmr1Pwm(TIM_CHANNEL_ALL, Pwm_STOP);
            tmr1PwmN(TIM_CHANNEL_ALL, Pwm_STOP);
            flags.risingBemfFlag = 0;
            commState = 0;
            break;
        }
        break;
    }
    commState++;
}

void tmr1Pwm(uint32_t channel, Pwm_e state)
{
    if (Pwm_START == state)
    {
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
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim1, channel);
    }
}

void tmr1PwmN(uint32_t channel, Pwm_e state)
{
    if (Pwm_START == state)
    {
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
    }
    else
    {
        HAL_TIMEx_PWMN_Stop(&htim1, channel);
    }
}

void tmr1SetCompare(uint32_t channel, uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(&htim1, channel, compare);
}

void tmr2SetCapturePolarity(uint32_t channel, uint32_t polarity)
{
    if (!(SensorType_SENSORED == settings.sensorType))
    {
        return;
    }
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, channel, polarity);
}
