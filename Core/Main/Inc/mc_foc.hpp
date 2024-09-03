#ifndef __PID_HPP
#define __PID_HPP

#include <stm32f3xx_hal.h>

namespace MC
{
    class PIDController
    {
    private:
        float kpGain;
        float kiGain;
        float kdGain;
        float outputLimit;
        float outputRamp;

        float integralTermSum;
        float pastError;
        float pastOutput;

    public:
        PIDController(float kpGain, float kiGain, float kdGain, float outputLimit, float outputRamp);
        void reset();
        float run(float error);
    };

    class FOC
    {
    private:
        /* data */
        uint8_t status;
    public:
        FOC(/* args */);
        void init();
        void innerLoop();
        void outerLoop();
    };
    
}

#endif // __PID_HPP
