#ifndef __MC_CONSTANTS_HPP
#define __MC_CONSTANTS_HPP


#define TIMER1_FREQUENCY            72000000
#define TIMER1_PRESCALER            0
#define PWM_FREQUENCY               18000
// PWM_MULTIPLIER = 1 for Up/Down counter mode
// PWM_MULTIPLIER = 2 for Center Aligned 1 counter mode
#define PWM_MULTIPLIER              2
#define PWM_PERIOD                  ((TIMER1_FREQUENCY / ((TIMER1_PRESCALER + 1) * PWM_FREQUENCY * PWM_MULTIPLIER)) - 1)
// +1 forces PWM to 100% duty cycle
#define MAX_DUTY_CYCLE              (PWM_PERIOD + 1)
#define TIMER2_FREQUENCY            72000000
#define TIMER2_PRESCALER            71
#define TMR2_COUNTS_PER_SEC         (TIMER2_FREQUENCY / (TIMER2_PRESCALER + 1))
#define MICROSECONDS_PER_SECOND     1000000
#define TMR2_COUNTS_PER_us          (TMR2_COUNTS_PER_SEC / MICROSECONDS_PER_SECOND)
// Resolution of TimebaseManager() in milliseconds
#define TIMEBASE_MS_PER_COUNT       10
// Number of milliseconds allowed to achieve magnetic lock
#define TIMEBASE_STARTUP_ms         2000
#define TIMEBASE_STARTUP_COUNT      (TIMEBASE_STARTUP_ms/TIMEBASE_MS_PER_COUNT)
// Number of milliseconds to dwell in warmup state
#define TIMEBASE_WARMUP_ms          400
#define TIMEBASE_WARMUP_COUNT       (TIMEBASE_WARMUP_ms / TIMEBASE_MS_PER_COUNT)
// Number of milliseconds in out-of-lock condition to recognize stall
#define TIMEBASE_STALL_ms           1000
#define TIMEBASE_STALL_COUNT        (TIMEBASE_STALL_ms / TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_10ms               10


// Number of permanent magnet poles.
// You can determine the number of poles by feeling the number of detents in one mechanical revolution and dividing that number by 3.
// NUM_POLES is always an even number and cannot be less than 2.
#define NUM_POLES                   8
// Number of phases in the motor
#define NUM_PHASES                  3
// Maximum speed in RPM
#define MAX_RPM                     9600
// Startup drive percentage
#define STARTUP_DRIVE_PCT           13
// Startup duty cycle
#define STARTUP_DUTYCYCLE           ((STARTUP_DRIVE_PCT * MAX_DUTY_CYCLE) / 100)
// Maximum sequential startup events before stop
#define MAX_STARTUP_EVENTS          2
// PWM duty cycle rate of change in response to speed control
// TIMEBASE_DUTY_RAMP * 10ms = time between steps
#define TIMEBASE_DUTY_RAMP          1
// Dwell time at slow step commutation
// TIMEBASE_SLOW_STEP * 10ms = dwell time
#define TIMEBASE_SLOW_STEP          20
// Number of slow commutations between warmup and startup
#define SLOW_STEPS                  1
// Stall commutation time in microseconds
#define STALL_COUNT_us              900
// maximum number that will come from ADC speed request
#define MAX_SPEED_REQUEST           4095
// percentage of speed request below which the motor will turn off
#define LOW_OFF_REQUEST_PCT         15
// percentage of speed request above which the motor will turn on
#define LOW_RESTORE_REQUEST_PCT     25
#define REQUEST_OFF                 ((MAX_SPEED_REQUEST * LOW_OFF_REQUEST_PCT) / 100)
#define REQUEST_ON                  ((MAX_SPEED_REQUEST * LOW_RESTORE_REQUEST_PCT) / 100)
// ADC averaging factor
// Number of samples in the ADC average = 2^ADC_AVG_FACTOR
#define ADC_AVG_FACTOR              2


#define M1_PWM_A_H                  TIM_CHANNEL_1
#define M1_PWM_B_H                  TIM_CHANNEL_2
#define M1_PWM_C_H                  TIM_CHANNEL_3
#define M1_PWM_A_L                  TIM_CHANNEL_1
#define M1_PWM_B_L                  TIM_CHANNEL_2
#define M1_PWM_C_L                  TIM_CHANNEL_3
#define M1_HALL_A                   TIM_CHANNEL_1
#define M1_HALL_B                   TIM_CHANNEL_2
#define M1_HALL_C                   TIM_CHANNEL_3
#define HALL_RISING                 TIM_INPUTCHANNELPOLARITY_RISING
#define HALL_FALLING                TIM_INPUTCHANNELPOLARITY_FALLING

#define ADC_BUFFER_ARRAY            adcBuffer
#define ADC_BUFFER_LENGTH           8
#define M1_CURR_FDBK_A              adcBuffer[0]
#define M1_V_BUS                    adcBuffer[1]
#define USR_POT                     adcBuffer[2]
#define M1_CURR_FDBK_C              adcBuffer[3]
#define M1_CURR_FDBK_B              adcBuffer[4]
#define M1_BEMF_A                   adcBuffer[5]
#define M1_BEMF_B                   adcBuffer[6]
#define M1_BEMF_C                   adcBuffer[7]


typedef enum
{
    ModulationType_LOW_SIDE,
    ModulationType_HIGH_SIDE,
} MC_ModulationType_e;

typedef enum
{
    Direction_CW,
    Direction_CCW,
} MC_Direction_e;

#endif // __MC_CONSTANTS_HPP
