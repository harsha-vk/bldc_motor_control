#include "mc_main.hpp"

void initSystem()
{
    // Stop interrupts and set counter to zero
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC2);
    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    // Stop pwm and set counter to zero
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    // Stop adc and clear adc buffer
    HAL_ADC_Stop_DMA(&hadc1);
    for (uint8_t i = 0; i < ADC_BUFFER_LENGTH; i++)
    {
        ADC_BUFFER_ARRAY[i] = 0;
    }

    // flashToData();

    timers.warmupTimer = TIMEBASE_WARMUP_COUNT;
    timers.slowStartTimer = TIMEBASE_SLOWSTART_COUNT;
    timers.stallTimer = TIMEBASE_STALL_COUNT;
    timers.dutyTimer = TIMEBASE_DUTY_COUNT;
    timers.pidTimer = TIMEBASE_PID_COUNT;
    timers.serialWriteTimer = TIMEBASE_SERIAL_COUNT;
    flags.tmrWarmupFlag = 0;
    flags.tmrSlowStartFlag = 0;
    flags.tmrStartupFlag = 0;
    flags.tmrStallFlag = 0;
    flags.tmrDutyFlag = 0;
    flags.tmrPidFlag = 0;
    flags.tmrSerialReadFlag = 0;
    flags.tmrSerialWriteFlag = 0;
    slowStartEvents = SLOW_STEPS;
    flags.stopFlag = 0;
    flags.runFlag = 0;
    flags.warmupCompleteFlag = 0;
    flags.initCompleteFlag = 0;
    flags.slowStartCompleteFlag = 0;
    flags.startupInProgress = 0;
    flags.startupCompleteFlag = 0;
    flags.stallFlag = 0;

    delete pidController;
    pidController = new MC::PIDController(mcParams.msg.kp_gain, mcParams.msg.ki_gain,
                                          mcParams.msg.kd_gain, 0, MAX_PWM_PULSE);

    // Start adc
    HAL_TIM_Base_Start(&htim1);
    HAL_ADC_Start_DMA(&hadc1, ADC_BUFFER_ARRAY, ADC_BUFFER_LENGTH);
}

void initDriver()
{
    mcData.msg.output_pulse = STARTUP_PULSE;
    stepNumber = 1;
    commutate();
    flags.startupInProgress = 1;
    flags.initCompleteFlag = 1;
}
