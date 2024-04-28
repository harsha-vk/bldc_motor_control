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
    uint8_t risingBemfFlag : 1;
    uint8_t measureBemfFlag : 1;
    uint8_t startupInProgress : 1;
} MC_Flags_t;

typedef struct
{
    MC_MotorControlMode_e motorControlMode : 4;
    MC_SensorType_e sensorType : 4;
    MC_ModulationType_e modulationType : 4;
    MC_Direction_e direction : 4;
} MC_Settings_t;

extern uint8_t commState;

extern uint16_t tmr2CommTime;
extern uint16_t zc;
extern uint16_t expectedZc;
extern uint16_t commAfterZc;

extern uint8_t slowStartEvents;
extern uint8_t tmrSlowStartTimer;
extern uint8_t tmrStartupTimer;
extern uint8_t tmrStallTimer;
extern uint8_t tmrWarmupTimer;
extern uint8_t tmrDutyTimer;
extern uint8_t tmrStallCheckTimer;
extern uint32_t timeBaseCount;

extern MC_Flags_t flags;

extern int zcError;
extern int temp;
extern int8_t ctemp;
extern uint8_t rampedSpeed;

extern uint32_t dutyCycle;
extern const uint32_t maxDutyCycle;

extern MC_Settings_t settings;

extern MC_IsrState_e isrState;
extern GPIO_TypeDef *gpioPort; // need to find alternate method
extern uint16_t gpioPin; // need to find alternate method

extern uint32_t adcVal[8];

void initSystem();
void initDriver();
void timeBaseManager();
void warmUpControl();
void controlSlowStart();
void controlStartUp();
void stallControl();
void speedManager();
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
