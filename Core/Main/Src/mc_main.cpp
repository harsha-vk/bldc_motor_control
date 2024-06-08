#include "application.hpp"
#include "mc_main.hpp"

MC_Settings_t settings;
MC_Timers_t timers;
MC_Flags_t flags;
uint8_t slowStartEvents;
uint32_t ADC_BUFFER_ARRAY[ADC_BUFFER_LENGTH];
uint8_t stepNumber;
MC::PIDController *pidController = new MC::PIDController(0, 0, 0, 0, 0);
_SerialTxMessage txData;
_SerialRxMessage rxData;

uint32_t timeBaseCount;

void serialToFlash();

void setup()
{
    flags.powerFlag = 0;
    flags.stopFlag = 1;
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);

}

void loop()
{
    if (flags.stopFlag)
    {
        initSystem();
    }
    timeBaseManager();
    warmUpControl();
    controlSlowStart();
    controlStartUp();
    stallControl();
    speedManager();
    pidManager();
    serialToFlash();

    // TODO: send readings to UART
}

void timeBaseManager()
{
    if ((timeBaseCount - HAL_GetTick()) >= TIMEBASE_10ms)
    {
        timeBaseCount = HAL_GetTick();
        flags.tmrWarmupFlag = 1;
        flags.tmrSlowStartFlag = 1;
        flags.tmrStartupFlag = 1;
        flags.tmrStallFlag = 1;
        flags.tmrDutyFlag = 1;
        flags.tmrPidFlag = 1;
        flags.tmrSerialReadFlag = 1;
    }
}

void warmUpControl()
{
    if (flags.warmupCompleteFlag || (!flags.tmrWarmupFlag))
    {
        return;
    }
    flags.tmrWarmupFlag = 0;
    if (timers.warmupTimer)
    {
        timers.warmupTimer--;
    }
    else
    {
        if (flags.runFlag)
        {
            flags.warmupCompleteFlag = 1;
            initDriver();
        }
    }
}

// TODO: Determine length of rxBuffer i.e. length of encoded rxData.
// Update code to flash rxData, accordingly.
void serialToFlash()
{
    if ((flags.powerFlag) || (!flags.tmrSerialReadFlag))
    {
        return;
    }
    uint8_t rxBuffer[RX_BUFFER_LENGTH] = {0};
    if (HAL_UART_Receive(&huart2, rxBuffer,
                         RX_BUFFER_LENGTH, RX_TIMEOUT) == HAL_OK)
    {
        pb_istream_t stream = pb_istream_from_buffer(rxBuffer, RX_BUFFER_LENGTH);

        if (pb_decode(&stream, SerialRxMessage_fields, &rxData))
        {
            uint32_t pageError;
            FLASH_EraseInitTypeDef flashEraseconfig;
            flashEraseconfig.TypeErase = FLASH_TYPEERASE_PAGES;
            flashEraseconfig.PageAddress = FLASH_PAGE_ADDRESS;
            flashEraseconfig.NbPages = 1;

            HAL_FLASH_Unlock();
            HAL_FLASHEx_Erase(&flashEraseconfig, &pageError);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_DATA_ADDRESS, 0);
            HAL_FLASH_Lock();
        }
    }
}
