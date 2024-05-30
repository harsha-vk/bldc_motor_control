#ifndef __READINGS_HPP
#define __READINGS_HPP

#include "stm32f3xx_hal.h"
#include "mc_constants.hpp"

namespace MC
{
    class Readings
    {
    private:
        uint16_t speedFdbk;

    public:
        MC_Rotate_e direction;
        uint16_t outputPulse;

        Readings();
        uint16_t getSpeedFdbk();
        void setSpeedFdbk(uint16_t cntVal);
        void toString();
    };

}

#endif // __READINGS_HPP
