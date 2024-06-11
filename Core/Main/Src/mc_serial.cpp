#include "mc_main.hpp"

void serialToData()
{
    if ((flags.powerFlag) || (!flags.tmrSerialReadFlag))
    {
        return;
    }
    flags.tmrSerialReadFlag = 0;
    uint8_t rxBuffer[McParamsMsg_size] = {0};
    if (HAL_UART_Receive(&huart2, rxBuffer,
                         McParamsMsg_size, 8) == HAL_OK)
    {
        pb_istream_t stream = pb_istream_from_buffer(rxBuffer, McParamsMsg_size);
        if (pb_decode(&stream, McParamsMsg_fields, &mcParams))
        {
            if (!mcParams.has_msg)
            {
                mcParams = McParamsMsg_init_zero;
                return;
            }
            // dataToFlash();
        }
    }
}

void dataToSerial()
{
    if ((!flags.startupCompleteFlag) || (!flags.tmrSerialWriteFlag))
    {
        return;
    }
    flags.tmrSerialWriteFlag = 0;

    if ((--timers.serialWriteTimer) == 0)
    {
        mcData.has_msg = true;
        mcData.msg.v_bus = ADC_M1_V_BUS;
        mcData.msg.curr_fdbk_a = ADC_M1_CURR_FDBK_A;
        mcData.msg.curr_fdbk_b = ADC_M1_CURR_FDBK_C;
        mcData.msg.curr_fdbk_c = ADC_M1_CURR_FDBK_C;
        mcData.msg.bemf_a = ADC_M1_BEMF_A;
        mcData.msg.bemf_b = ADC_M1_BEMF_B;
        mcData.msg.bemf_c = ADC_M1_BEMF_C;
        // TODO: If necessary, include mcParams.msg within mcData.

        uint8_t txBuffer[McDataMsg_size] = {0};
        pb_ostream_t stream = pb_ostream_from_buffer(txBuffer, McDataMsg_size);
        if (pb_encode(&stream, McDataMsg_fields, &mcData))
        {
            HAL_UART_Transmit(&huart2, txBuffer, McDataMsg_size, 8);
        }
        timers.serialWriteTimer = TIMEBASE_SERIAL_COUNT;
    }
}
