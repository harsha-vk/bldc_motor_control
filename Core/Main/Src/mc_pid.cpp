#include "mc_pid.hpp"

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

    error = reference - fdbk;

    wProportionalTerm = kpGain * error;

    if (0 == kiGain)
    {
        integralTermSum = 0;
    }
    else
    {
        wIntegralTerm = kiGain * error;
        integralTermSum += wIntegralTerm;
    }

    if (integralTermSum > ((int32_t)upperLimitOutput << GAIN_DIV))
    {
        integralTermSum = (int32_t)upperLimitOutput << GAIN_DIV;
    }
    else if (integralTermSum < ((int32_t)lowerLimitOutput << GAIN_DIV))
    {
        integralTermSum = (int32_t)lowerLimitOutput << GAIN_DIV;
    }

    wDifferentialTerm = kdGain * (fdbk - lastFdbk);
    lastFdbk = fdbk;

    wOutput32 = (wProportionalTerm >> GAIN_DIV) + (integralTermSum >> GAIN_DIV) - (wDifferentialTerm >> GAIN_DIV);

    if (wOutput32 > upperLimitOutput)
    {
        wOutput32 = upperLimitOutput;
    }
    else if (wOutput32 < lowerLimitOutput)
    {
        wOutput32 = lowerLimitOutput;
    }

    return ((int16_t)(wOutput32));
}
