#include "readings.hpp"

MC::Readings::Readings()
{
    ;
}

void MC::Readings::setSpeedFdbk(uint16_t cntVal)
{
    // Electrical frequency in Hz
    uint16_t elFreqHz = TMR2_COUNTS_PER_SEC / (cntVal * 6);
    // Equivalent mechanical speed in RPM
    uint16_t mechSpeedRpm = elFreqHz * 60 / POLE_PAIRS;
    // Weighted moving average
    this->speedFdbk = (this->speedFdbk * 10 / 100) + (mechSpeedRpm * (100 - 10) / 100);
}

void MC::Readings::toString()
{
    ;
}