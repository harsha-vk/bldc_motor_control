#ifndef MC_MAIN_H
#define MC_MAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Regular ADC group */
typedef enum
{
    MC_ADC_REG_I_A = 0,
    MC_ADC_REG_V_BUS,
    MC_ADC_REG_POT,
    MC_ADC_REG_I_C,
    MC_ADC_REG_I_B,
    MC_ADC_REG_COUNT
} mc_adc_reg_idx_t;

/* Injected ADC group */
typedef enum
{
    MC_ADC_INJ_BEMF_A = 0,
    MC_ADC_INJ_BEMF_C,
    MC_ADC_INJ_BEMF_B,
    MC_ADC_INJ_COUNT
} mc_adc_inj_idx_t;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

extern uint32_t mc_adc_reg_buf[MC_ADC_REG_COUNT];

extern mc_config_t mc_config;

static inline uint32_t mc_adc_read(mc_adc_reg_idx_t idx)
{
    return ((volatile uint32_t *)mc_adc_reg_buf)[idx];
}

void mc_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MC_MAIN_H */
