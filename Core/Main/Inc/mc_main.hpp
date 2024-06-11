#ifndef __MC_MAIN_HPP
#define __MC_MAIN_HPP

#include "main.h"
#include "mc_constants.hpp"
#include "mc_pid.hpp"
#include "pb_encode.h"
#include "pb_decode.h"
#include "serial.pb.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

typedef struct
{
    uint8_t warmupTimer;
    uint8_t slowStartTimer;
    uint8_t startupTimer;
    uint8_t stallTimer;
    uint8_t dutyTimer;
    uint8_t pidTimer;
    uint8_t serialWriteTimer;
} MC_Timers_t;

typedef struct
{
    uint8_t tmrWarmupFlag : 1;
    uint8_t tmrSlowStartFlag : 1;
    uint8_t tmrStartupFlag : 1;
    uint8_t tmrStallFlag : 1;
    uint8_t tmrDutyFlag : 1;
    uint8_t tmrPidFlag : 1;
    uint8_t tmrSerialReadFlag : 1;
    uint8_t tmrSerialWriteFlag : 1;
    uint8_t powerFlag : 1;
    uint8_t stopFlag : 1;
    uint8_t runFlag : 1;
    uint8_t warmupCompleteFlag : 1;
    uint8_t initCompleteFlag : 1;
    uint8_t slowStartCompleteFlag : 1;
    uint8_t startupInProgress : 1;
    uint8_t startupCompleteFlag : 1;
    uint8_t stallFlag : 1;
} MC_Flags_t;

extern MC_Timers_t timers;
extern MC_Flags_t flags;
extern uint8_t slowStartEvents;
extern uint32_t ADC_BUFFER_ARRAY[ADC_BUFFER_LENGTH];
extern uint8_t stepNumber;
extern uint32_t activeHallChannel;
extern MC::PIDController *pidController;
extern McDataMsg mcData;
extern McParamsMsg mcParams;

void initSystem();
void initDriver();
void timeBaseManager();
void warmUpControl();
void controlSlowStart();
void controlStartUp();
void commutate();
void stallControl();
void speedManager();
void pidManager();
void serialToData();
void dataToSerial();
void flashToData();
void dataToFlash();

#endif // __MC_MAIN_HPP
