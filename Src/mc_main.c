#include "mc_main.h"
#include "mc_types.h"
#include "main.h"
#include "stm32f3xx_ll_adc.h"

#include <stdio.h>
#include <string.h>

uint32_t mc_adc_buf[MC_ADC_COUNT];

mc_config_t mc_config;

static void mc_pwm_disable(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

static void mc_adc_start(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start_DMA(&hadc1, mc_adc_buf, MC_ADC_COUNT) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void mc_i_sense_reset(void)
{
    /* TODO */
}

static void mc_load_config(void)
{
    const mc_config_t *config = (const mc_config_t *)MC_CONFIG_BASE;

    if ((uint32_t)config->mode >= MC_MODE_SENSORED_SIX_STEP
        && (uint32_t)config->mode <= MC_MODE_MOTOR_CALIBRATION)
    {
        memcpy(&mc_config, config, sizeof(mc_config));
        printf("Config loaded from flash\n");
    }
    else
    {
        memset(&mc_config, 0, sizeof(mc_config));
        mc_config.mode = MC_MODE_SENSORED_SIX_STEP;
        printf("No valid config in flash, using defaults\n");
    }
}

void mc_init(void)
{
    printf("\nBLDC Motor Control\n");

    mc_pwm_disable();

    mc_adc_start();

    mc_i_sense_reset();

    mc_load_config();
}
