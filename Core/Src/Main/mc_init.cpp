#include "application.hpp"

void initSystem()
{
    // Stop interrupts
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC2);
    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2,0);
    // Stop pwm
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_ALL);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_ALL);
    // Stop adc
    HAL_TIM_Base_Stop(&htim1);
    HAL_ADC_Stop_DMA(&hadc1);

    settings.direction = Direction_CW;
    settings.modulationType = ModulationType_HIGH_SIDE;

    timers.warmupTimer = TIMEBASE_WARMUP_COUNT;
    timers.slowStartTimer = TIMEBASE_SLOW_STEP;
    timers.dutyTimer = TIMEBASE_DUTY_RAMP;
    timers.stallTimer = TIMEBASE_STALL_COUNT;
    flags.tmrStartupFlag = 0;
    flags.tmrWarmupFlag = 0;
    flags.tmrStallFlag = 0;
    flags.tmrSlowStartFlag = 0;
    flags.tmrDutyFlag = 0;
    slowStartEvents = SLOW_STEPS;
    flags.warmupCompleteFlag = 0;
    flags.startupCompleteFlag = 0;
    flags.slowStartCompleteFlag = 0;
    flags.stallFlag = 0;
    flags.startupInProgress = 0;
    flags.stopFlag = 0;
    flags.runFlag = 0;

    // Start adc
    HAL_TIM_Base_Start(&htim1);
    HAL_ADC_Start_DMA(&hadc1, ADC_BUFFER_ARRAY, ADC_BUFFER_LENGTH);

    flags.initCompleteFlag = 0;
}

void initDriver()
{
    dutyCycle = STARTUP_DUTYCYCLE;
    commState = 1;
    commutate();
    flags.startupInProgress = 1;
    flags.initCompleteFlag = 1;
}
