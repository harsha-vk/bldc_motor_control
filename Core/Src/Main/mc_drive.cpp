#include "application.hpp"

typedef enum : uint8_t
{
    Pwm_STOP,
    Pwm_START
} Pwm_e;

GPIO_TypeDef *gpioPort;
uint16_t gpioPin;

void tmr1Pwm(uint32_t channel, Pwm_e state);
void tmr1PwmN(uint32_t channel, Pwm_e state);
void tmr1SetCompare(uint32_t channel, uint32_t compare);
void tmr2InputCaptureStartInterrupt(uint32_t channel, uint32_t polarity);

void commutate()
{
    switch (settings.motorControlMode)
    {
    case MotorControlMode_6STEP:
        switch (commState)
        {
        case 1:
            tmr1Pwm(M1_PWM_C_H, Pwm_STOP);
            tmr1Pwm(M1_PWM_A_H, Pwm_START);
            tmr1PwmN(M1_PWM_B_L, Pwm_START);
            tmr2InputCaptureStartInterrupt(M1_HALL_C, HALL_FALLING);
            gpioPort = M1_HALL_C_GPIO_Port; // need to find alternate method
            gpioPin = M1_HALL_C_Pin;
            flags.risingBemfFlag = 0;
            break;
        case 2:
            tmr1PwmN(M1_PWM_B_L, Pwm_STOP);
            // tmr1Pwm(M1_PWM_A_H, Pwm_START);
            tmr1PwmN(M1_PWM_C_L, Pwm_START);
            tmr2InputCaptureStartInterrupt(M1_HALL_B, HALL_RISING);
            gpioPort = M1_HALL_B_GPIO_Port;
            gpioPin = M1_HALL_B_Pin;
            flags.risingBemfFlag = 1;
            break;
        case 3:
            tmr1Pwm(M1_PWM_A_H, Pwm_STOP);
            tmr1Pwm(M1_PWM_B_H, Pwm_START);
            // tmr1PwmN(M1_PWM_C_L, Pwm_START);
            tmr2InputCaptureStartInterrupt(M1_HALL_A, HALL_FALLING);
            gpioPort = M1_HALL_A_GPIO_Port;
            gpioPin = M1_HALL_A_Pin;
            flags.risingBemfFlag = 0;
            break;
        case 4:
            tmr1PwmN(M1_PWM_C_L, Pwm_STOP);
            // tmr1Pwm(M1_PWM_B_H, Pwm_START);
            tmr1PwmN(M1_PWM_A_L, Pwm_START);
            tmr2InputCaptureStartInterrupt(M1_HALL_C, HALL_RISING);
            gpioPort = M1_HALL_C_GPIO_Port;
            gpioPin = M1_HALL_C_Pin;
            flags.risingBemfFlag = 1;
            break;
        case 5:
            tmr1Pwm(M1_PWM_B_H, Pwm_STOP);
            tmr1Pwm(M1_PWM_C_L, Pwm_START);
            // tmr1PwmN(M1_PWM_A_L, Pwm_START);
            tmr2InputCaptureStartInterrupt(M1_HALL_B, HALL_FALLING);
            gpioPort = M1_HALL_B_GPIO_Port;
            gpioPin = M1_HALL_B_Pin;
            flags.risingBemfFlag = 0;
            break;
        case 6:
            tmr1PwmN(M1_PWM_A_L, Pwm_STOP);
            tmr1Pwm(M1_PWM_C_H, Pwm_START);
            tmr1PwmN(M1_PWM_B_L, Pwm_START);
            tmr2InputCaptureStartInterrupt(M1_HALL_A, HALL_RISING);
            gpioPort = M1_HALL_A_GPIO_Port;
            gpioPin = M1_HALL_A_Pin;
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
    // write commstate code for ccw direction as well 
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

void tmr2InputCaptureStartInterrupt(uint32_t channel, uint32_t polarity)
{
    if (!(SensorType_SENSORED == settings.sensorType))
    {
        return;
    }
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, channel, polarity);
    if (!(flags.startupCompleteFlag))
    {
        return;
    }
    HAL_TIM_IC_Start_IT(&htim2, channel);
}
