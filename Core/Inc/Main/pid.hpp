#ifndef __PID_HPP
#define __PID_HPP

#include "stm32f3xx_hal.h"

namespace MC
{
    class PIDController
    {
    private:
        const int16_t GAIN_DIV = 8192;
        int16_t reference;
        int16_t kpGain;
        int16_t kiGain;
        int16_t kdGain;
        int16_t lowerLimitOutput;
        int16_t upperLimitOutput;
        int32_t integralTermSum;
        int16_t lastFdbk;

    public:
        PIDController(int16_t kpGain, int16_t kiGain, int16_t kdGain,
                  int16_t lowerLimitOutput, int16_t upperLimitOutput);
        void setReference(uint16_t reference);
        int16_t updateOutput(int16_t fdbk);
    };
}

#endif // __PID_HPP
