#include "pid.hpp"

MC::PIDController::PIDController(int16_t kpGain, int16_t kiGain, int16_t kdGain,
                                 int16_t lowerLimitOutput, int16_t upperLimitOutput)
{
    this->reference = 0;
    this->kpGain = kpGain;
    this->kiGain = kiGain;
    this->kdGain = kdGain;
    this->lowerLimitOutput = lowerLimitOutput;
    this->upperLimitOutput = upperLimitOutput;
    this->integralTermSum = 0;
}

void MC::PIDController::setReference(uint16_t reference)
{
    this->reference = reference;
}

int16_t MC::PIDController::updateOutput(int16_t fdbk)
{
    int32_t wProportionalTerm = 0, wIntegralTerm = 0, wDifferentialTerm = 0, wOutput32 = 0;
    int32_t error = 0;

    error = this->reference - fdbk;

    wProportionalTerm = this->kpGain * error;

    wIntegralTerm = this->kiGain * error;
    this->integralTermSum += wIntegralTerm;

    if (this->integralTermSum > this->GAIN_DIV * this->upperLimitOutput)
    {
        this->integralTermSum = this->GAIN_DIV * this->upperLimitOutput;
    }
    if (this->integralTermSum < -this->GAIN_DIV * this->upperLimitOutput)
    {
        this->integralTermSum = -this->GAIN_DIV * this->upperLimitOutput;
    }

    wDifferentialTerm = this->kdGain * (fdbk - this->lastFdbk);
    this->lastFdbk = fdbk;

    wOutput32 = (wProportionalTerm / GAIN_DIV) + (this->integralTermSum / GAIN_DIV) - (wDifferentialTerm / GAIN_DIV);

    if (wOutput32 > this->upperLimitOutput)
    {
        wOutput32 = this->upperLimitOutput;
    }
    else if (wOutput32 < this->lowerLimitOutput)
    {
        wOutput32 = this->lowerLimitOutput;
    }

    return ((int16_t)(wOutput32));
}
