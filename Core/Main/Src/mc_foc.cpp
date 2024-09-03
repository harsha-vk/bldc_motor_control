#include "mc_foc.hpp"

MC::PIDController::PIDController(float kpGain, float kiGain, float kdGain, float outputLimit, float outputRamp)
    : kpGain(kpGain),
      kiGain(kiGain),
      kdGain(kdGain),
      outputLimit(outputLimit),
      outputRamp(outputRamp),
      pastError(0.0f),
      pastOutput(0.0f),
      integralTermSum(0.0f)
{
    ;
}

void MC::PIDController::reset()
{
    pastError = 0;
    pastOutput = 0;
    integralTermSum = 0;
}

float MC::PIDController::run(float error)
{
    float proportionalTerm = 0, integralTerm = 0, derivativeTerm = 0, output = 0;

    proportionalTerm = kpGain * error;

    if (0 == kiGain)
    {
        integralTermSum = 0;
    }
    else
    {
        integralTerm = kiGain * error;
        integralTermSum += integralTerm;
    }

    if (integralTermSum > outputLimit)
    {
        integralTermSum = outputLimit;
    }
    else if (integralTermSum < (-1.0f * outputLimit))
    {
        integralTermSum = (-1.0f * outputLimit);
    }

    derivativeTerm = kdGain * error;

    output = proportionalTerm + integralTerm + derivativeTerm;

    if (output > outputLimit)
    {
        output = outputLimit;
    }
    else if (output < outputLimit)
    {
        output = (-1.0f * outputLimit);
    }

    if (outputRamp > 0)
    {
        float outputDiff = output - pastOutput;
        if (outputDiff > outputRamp)
        {
            output = pastOutput + outputRamp;
        }
        else if (outputDiff < (-1.0f * outputRamp))
        {
            output = pastOutput - outputRamp;
        }
    }

    return output;
}
