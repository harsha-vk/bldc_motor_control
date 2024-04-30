#ifndef __APPLICATION_HPP
#define __APPLICATION_HPP

#include "main.h"
#include "stm32f3xx_hal.h"
#include "mc_constants.hpp"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

typedef struct
{
    uint8_t slowStartTimer;
    uint8_t startupTimer;
    uint8_t stallTimer;
    uint8_t warmupTimer;
    uint8_t dutyTimer;
} MC_Timers_t;

typedef struct
{
    uint8_t tmrStartupFlag : 1;
    uint8_t tmrWarmupFlag : 1;
    uint8_t tmrSlowStartFlag : 1;
    uint8_t tmrStallFlag : 1;
    uint8_t tmrDutyFlag : 1;
    uint8_t warmupCompleteFlag : 1;
    uint8_t startupCompleteFlag : 1;
    uint8_t slowStartCompleteFlag : 1;
    uint8_t stallFlag : 1;
    uint8_t stallRecoveryFlag : 1;
    uint8_t stopFlag : 1;
    uint8_t runFlag : 1;
    uint8_t initCompleteFlag : 1;
    uint8_t startupInProgress : 1;
} MC_Flags_t;

typedef struct
{
    MC_ModulationType_e modulationType : 4;
    MC_Direction_e direction : 4;
} MC_Settings_t;

extern MC_Settings_t settings;
extern MC_Timers_t timers;
extern MC_Flags_t flags;
extern uint8_t slowStartEvents;
extern uint32_t ADC_BUFFER_ARRAY[ADC_BUFFER_LENGTH];
extern uint32_t dutyCycle;
extern uint8_t commState;

extern uint32_t activeHallChannel;

extern uint32_t tmr2CommCnt;

void initSystem();
void initDriver();
void timeBaseManager();
void warmUpControl();
void controlSlowStart();
void controlStartUp();
void stallControl();
void speedManager();
void speedFeedback();
void commutate();

#ifdef __cplusplus
extern "C"
{
#endif

    void setup();
    void loop();

#ifdef __cplusplus
}
#endif

#endif
