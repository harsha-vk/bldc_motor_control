#ifndef MC_TYPES_H
#define MC_TYPES_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "pid.h"

#define FLASH_PAGE_31_BASE 0x0800F800UL
#define MC_CONFIG_BASE     FLASH_PAGE_31_BASE

typedef enum
{
    MC_MODE_SENSORED_SIX_STEP = 1,
    MC_MODE_SENSORLESS_SIX_STEP,
    MC_MODE_SENSORED_FOC,
    MC_MODE_MOTOR_CALIBRATION
} mc_mode_t;

typedef struct
{
    uint8_t pp;     /* Pole pairs */
    float v_rated;  /* Rated voltage (V) */
    float i_rated;  /* Rated current (A) */
    float i_peak;   /* Peak current (A) */
    uint32_t n_max; /* Maximum speed (RPM) */
    float rs;       /* Phase resistance (ohm) */
    float ls;       /* Phase inductance (mH) */
    float kt;       /* Torque constant (Nm/A) */
} mc_motor_params_t;

typedef struct
{
    pid_params_t id_params;    /* d-axis current loop: in A, out V */
    pid_params_t iq_params;    /* q-axis current loop: in A, out V */
    pid_params_t speed_params; /* speed loop: in speed, out Iq ref (A) */
} mc_foc_params_t;

typedef struct
{
    pid_params_t speed_params; /* speed loop: in speed, pulse */
} mc_6step_params_t;

typedef struct
{
    uint32_t RES;
    mc_mode_t mode;
    mc_motor_params_t motor_params;
    mc_foc_params_t foc_params;
    mc_6step_params_t six_step_params;
} mc_config_t;

#ifdef __cplusplus
}
#endif

#endif /* MC_TYPES_H */
