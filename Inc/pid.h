#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    float kp;      /* Proportional gain */
    float ki;      /* Integral gain */
    float kd;      /* Derivative gain */
    float out_min; /* Output lower limit */
    float out_max; /* Output upper limit */
} pid_params_t;

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
