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
    MC_ADC_BEMF_A = 0, /* IN9  */
    MC_ADC_BEMF_B,     /* IN14 */
    MC_ADC_BEMF_C,     /* IN13 */
    MC_ADC_I_A,        /* IN1  */
    MC_ADC_I_B,        /* IN7  */
    MC_ADC_I_C,        /* IN6  */
    MC_ADC_V_BUS,      /* IN2  */
    MC_ADC_POT,        /* IN5  */
    MC_ADC_COUNT
} mc_adc_idx_t;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

extern uint32_t mc_adc_buf[MC_ADC_COUNT];

extern mc_config_t mc_config;

static inline uint32_t mc_adc_read(mc_adc_idx_t idx)
{
    return ((volatile uint32_t *)mc_adc_buf)[idx];
}

void mc_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MC_MAIN_H */
