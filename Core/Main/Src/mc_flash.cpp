#include "mc_main.hpp"

// Get motor control params from flash
void flashToData()
{
    uint8_t fBuffer[sizeof(mcParams)] = {0};
    for (uint32_t i = 0; i < sizeof(mcParams); i++)
    {
        uint64_t fAddress = FLASH_DATA_ADDRESS + i;
        *((uint8_t *)fBuffer + i) = *(uint8_t *)fAddress;
    }
    memcpy(&mcParams, fBuffer, sizeof(mcParams));
    if (!mcParams.has_msg)
    {
        mcParams = McParamsMsg_init_zero;
    }
}

// Save motor control params to flash
void dataToFlash()
{
    uint32_t pageError;
    FLASH_EraseInitTypeDef flashEraseconfig;
    flashEraseconfig.TypeErase = FLASH_TYPEERASE_PAGES;
    flashEraseconfig.PageAddress = FLASH_PAGE_ADDRESS;
    flashEraseconfig.NbPages = 1;

    uint8_t fSize64 = (sizeof(mcParams) + sizeof(uint64_t) - 1) / sizeof(uint64_t);
    uint64_t fBuffer[fSize64] = {0};
    memcpy(fBuffer, &mcParams, sizeof(mcParams));

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flashEraseconfig, &pageError);
    for (uint8_t i = 0; i < fSize64; i++)
    {
        uint64_t fAddress = FLASH_DATA_ADDRESS + (i * sizeof(uint64_t));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, fAddress, fBuffer[i]);
    }
    HAL_FLASH_Lock();
}
