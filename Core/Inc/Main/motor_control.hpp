#ifndef __MOTOR_CONTROL_HPP
#define __MOTOR_CONTROL_HPP

// Timer Definitions
#define TIMER1_FREQUENCY            72000000
#define TIMER1_PRESCALER            0
#define PWM_FREQUENCY               18000
// 1 for Up/Down counter mode
// 2 for Center Aligned 1 counter mode
#define PWM_MULTIPLIER              2
#define PWM_PERIOD                  (((TIMER1_FREQUENCY / ((TIMER1_PRESCALER + 1) * PWM_FREQUENCY * PWM_MULTIPLIER)) - 1) & 0xFFFF)
// +1 forces PWM to 100% duty cycle
#define MAX_DUTY_CYCLE              ((PWM_PERIOD + 1) & 0xFFFF)
#define TIMER2_FREQUENCY            72000000
#define TIMER2_PRESCALER            71
#define TMR2_COUNTS_PER_SEC         (TIMER2_FREQUENCY / (TIMER2_PRESCALER + 1))
#define MICROSECONDS_PER_SECOND     1000000
#define TMR2_COUNTS_PER_us          (TMR2_COUNTS_PER_SEC / MICROSECONDS_PER_SECOND)
// Resolution of TimebaseManager() in milliseconds
#define TIMEBASE_MS_PER_COUNT       10
// Number of milliseconds allowed to achieve zero-cross lock
#define TIMEBASE_STARTUP_ms         2000
#define TIMEBASE_STARTUP_COUNT      (TIMEBASE_STARTUP_ms/TIMEBASE_MS_PER_COUNT)
// Number of milliseconds to dwell in warmup state
#define TIMEBASE_WARMUP_ms          400
#define TIMEBASE_WARMUP_COUNT       (TIMEBASE_WARMUP_ms / TIMEBASE_MS_PER_COUNT)
// Number of milliseconds between each motor stall condition check
#define TIMEBASE_STALLCHECK_ms      500
#define TIMEBASE_STALLCHECK_COUNT   (TIMEBASE_STALLCHECK_ms / TIMEBASE_MS_PER_COUNT)
// Number of milliseconds in out-of-lock condition to recognize stall
#define TIMEBASE_STALL_ms           1000
#define TIMEBASE_STALL_COUNT        (TIMEBASE_STALL_ms / TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_10ms               10

// Parameters that should be adjusted to match the motor being used:
//
// NUM_POLES - This is the number of magnetic poles in the motor's permanent magnet
//             You can determine the number of poles by feeling the number of
//             detents in one mechanical revolution and dividing that number by 3.
//             NUM_POLES is always an even number and cannot be less than 2.
//
// START_RPM - This determines the open-loop commutation time at startup. A good
//             starting point is 1/10th the expected motor speed at the startup voltage.
//
// STARTUP_DUTY_CYCLE - This is the startup duty cycle which determines the startup
//             voltage. Voltage = AppliedVoltage * Duty_cycle_count/MAX_DUTY_CYCLE.
//             Example: Applied voltage = 80V, STARTUP_DUTY_CYCLE = 0x10, therefore:
//             Startup voltage = 80 * 16/125 = 10.24 volts.
//
// MAX_STARTUP_EVENTS - The startup algorithm starts by single stepping the motor
//             this number of times. This is done to position the motor in a known
//             alignment before higher speed commutation is attempted. At least two
//             steps are needed because at the first step the motor may be prepositioned
//             in a state where it cannot swing left or right because it's in the middle
//             of that boundary. The second step assures that the motor will respond.
//
// TIMEBASE_SLOW_STEP - The time to dwell at each single step commutation during pre-
//             start. Some motors have very high inertia to overcome. The slow step
//             time should be only large enough to allow the motor to reach and stabilize
//             at each slow start commutation.
//
// TIMEBASE_DUTY_RAMP - At startup the applied motor voltage is set to the minimum.
//             The applied voltage starts to ramp up after commutation lock is detected
//             at the startup voltage. TIMEBASE_DUTY_RAMP determines the time to dwell
//             at each ramp-up step thereby slowing the rate at which the applied voltage
//             is increased. High inertia systems need more time than low inertia systems.
//
// BLANKING_COUNT_us - The number of microseconds to hold off from detecting zero cross
//             immediately after a commutation event. This allows the flyback currents
//             to die out so the back EMF can be accurately measured. Flyback current
//             causes the large spike in the back EMF voltage immediately after releasing
//             the motor phase from active drive current.
//
// STALL_COUNT_us - This is the shortest expected commutation time. When the motor stalls
//             a false zero cross is detected immediately after every commutation. This causes
//             the zero cross tracking algorithm to repeatedly decrease the commutation time
//             at each commutation event until the commutation becomes shorter than the motor's
//             top rated speed. The control loop recognizes this and reverts back to the
//             startup procedure when it happens.
//
// RAMP_INCR - This is used only for main motor drive programs that include high-inertia
//             startup. During the open-loop phase of startup, RAMP_INCR determines the next
//             commutation time as a percentage of the previous commutation time. Larger
//             system inertia requires smaller percentage.
//
// ERROR_SCALE - Feedback scaling factor.
//             The error is the difference between the expected zero cross time and the
//             actual zero cross time. The error is scaled down before it is accumulated into
//             into the commutation time. 2 raised to ERROR_SCALE (2^ERROR_SCALE) is the scaling
//             division factor. If the scaling factor is too large then the motor response will
//             be slow. If the scaling factor is too small then the motor may become unstable.
//             If the motor frequently misses lock after the startup sequence then this number is
//             probably too small. If the motor loses lock at high speed or during acceleratioin
//             then this number is probably too large.
//
// HIGH_INERTIA - Define this variable for systems with slow response times.

// Motor Definitions

// Number of permanent magnet poles
#define NUM_POLES                   8
// Number of phases in the motor
#define NUM_PHASES                  3
// Number of commutations in one mechanical revolution
#define COMM_PER_REV                (NUM_POLES * NUM_PHASES)
// Start speed in RPM
#define START_RPM                   130
// 60 seconds in one minute
#define SEC_PER_MIN                 60
// Startup commutations per minute
#define START_COMM_PER_MIN          (START_RPM * COMM_PER_REV)
// Commutations per second at beginning of startup ramp
#define START_COMM_PER_SEC          (START_COMM_PER_MIN / SEC_PER_MIN)
// Timer2 counts per commutation at beginning of startup
#define TMR2_START_COUNT            (TMR2_COUNTS_PER_SEC / START_COMM_PER_SEC)
// Initial starup commutation period
#define COMM_TIME_INIT              TMR2_START_COUNT
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
// Startup ramp - specifies ramp step as percentage of comm time in number of right shifts.
// Example: >> 6 = 1 / 64 = .0156 = 1.56% step. This minimizes the 1 / X effect of a fixed step.
#define RAMP_INCR                   6
// Blanking count in microseconds
#define BLANKING_COUNT_us           100
#define BLANKING_COUNT              (BLANKING_COUNT_us * TMR2_COUNTS_PER_us)
// Stall commutation time in microseconds
#define STALL_COUNT_us              900
// Number of Timer2 counts below which a stall condition is detected
#define MIN_COMM_TIME               ((STALL_COUNT_us * TMR2_COUNTS_PER_SEC) / MICROSECONDS_PER_SECOND)
// The raw error is divided by 2 to the power of ERROR_SCALE before accumulating.
// Example: If the raw error is 96 and ERROR_SCALE is 3 then the error correction that is accumulated
// is reduced to 96/2^3 or 12.
#define ERROR_SCALE                 3
// Number of microseconds to commutate early after zero cross
// this number is subtracted from half the expected commutation time to set commutation event after zero cross
#define ADVANCE_TIMING_us           0
// Commutation happens in two stages
// Stage 1 is zero cross detection: Commutation is forced 1/2 commutation period after the Z-C event.
// Stage 2 is fixed commutation: Commutation timer is set at beginning for full commutation period.
// Stage 1 takes 12 us to detect Z-C and restuff the commutation timer. FIXED_ADVANCE_us adjusts for that difference.
#define FIXED_ADVANCE_TIMING_us     (ADVANCE_TIMING_us - 0)
#define ADVANCE_COUNT               (ADVANCE_TIMING_us * TMR2_COUNTS_PER_us)
#define FIXED_ADVANCE_COUNT         (FIXED_ADVANCE_TIMING_us * TMR2_COUNTS_PER_us)

// maximum number that will come from ADC speed request
#define MAX_SPEED_REQUEST           (4095)
// percentage of speed request below which the motor will turn off
#define LOW_OFF_REQUEST_PCT         (15)
// percentage of speed request above which the motor will turn on
#define LOW_RESTORE_REQUEST_PCT     (25)
#define REQUEST_OFF                 ((MAX_SPEED_REQUEST * LOW_OFF_REQUEST_PCT) / 100)
#define REQUEST_ON                  ((MAX_SPEED_REQUEST * LOW_RESTORE_REQUEST_PCT) / 100)
// ADC averaging factor
// Number of samples in the ADC average = 2^ADC_AVG_FACTOR
#define ADC_AVG_FACTOR              (2)

#define STARTUP_COUNT               (0xFFFF - COMM_TIME_INIT + 1)
#define MAX_TMR2_PRESET             (0xFFFF - MIN_COMM_TIME)

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

#endif
