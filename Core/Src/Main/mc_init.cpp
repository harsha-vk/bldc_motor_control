#include "application.hpp"

void initSystem()
{
    // Stop interrupts
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Stop_IT(&htim2);
    // Stop pwm
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_ALL);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Stop(&htim1);
    // Stop adc
    HAL_TIM_Base_Stop(&htim1);
    HAL_ADC_Stop_DMA(&hadc1);

    tmrWarmupTimer = TIMEBASE_WARMUP_COUNT;
    tmrSlowStartTimer = TIMEBASE_SLOW_STEP;
    tmrDutyTimer = TIMEBASE_DUTY_RAMP;
    tmrStallCheckTimer = TIMEBASE_STALLCHECK_COUNT;
    tmrStallTimer = TIMEBASE_STALL_COUNT;
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

    flags.measureBemfFlag = 0;
    zcError = 0;
    temp = 0;
    ctemp = 0;
    expectedZc = 0;
    zc = 0;
    commAfterZc = 0;
    rampedSpeed = 0;

    settings.motorControlMode = MotorControlMode_6STEP;
    settings.sensorType = SensorType_SENSORED;
    settings.modulationType = ModulationType_HIGH_SIDE; // Default: HIGH_SIDE

    if ((MotorControlMode_6STEP == settings.motorControlMode) &&
        (SensorType_SENSORLESS_1 == settings.sensorType) &&
        (ModulationType_LOW_SIDE == settings.modulationType))
    {
        TIM_IC_InitTypeDef sConfigIC = {0};
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
        sConfigIC.ICFilter = 0;
        HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
        HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);
        HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);
    }

    // Start adc
    HAL_TIM_Base_Start(&htim1);
    HAL_ADC_Start_DMA(&hadc1, adcVal, 8);

    flags.initCompleteFlag = 0;
}

void initDriver()
{
    dutyCycle = STARTUP_DUTYCYCLE;

    tmr2CommTime = STARTUP_COUNT;
    expectedZc = (tmr2CommTime >> 1) | 0x8000;
    commState = 1;
    commutate();
    flags.startupInProgress = 1;
    flags.initCompleteFlag = 1;
}
