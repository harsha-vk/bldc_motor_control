#include "application.hpp"

long adcMap(long x, long inMin, long inMax, long outMin, long outMax);

void SpeedManager(void)
{
	static uint32_t srsum = 0;
	static uint32_t sravg = 0;

	if (flags.tmrDutyFlag)
	{
		return;
	}
	flags.tmrDutyFlag = 0;

	if (--tmrDutyTimer)
	{
		return;
	}
	tmrDutyTimer = TIMEBASE_DUTY_RAMP;

	srsum -= sravg;
	srsum += adcVal[1];

	sravg = srsum >> ADC_AVG_FACTOR;

	if (sravg < REQUEST_OFF)
	{
		if (flags.runFlag)
		{
			flags.stopFlag = 1;
		}
		flags.runFlag = 0;
		return;
	}
	if (sravg > REQUEST_ON)
	{
		flags.runFlag = 1;
	}
	if (!flags.startupCompleteFlag)
	{
		return;
	}

	uint32_t val = adcMap(sravg, 0, MAX_SPEED_REQUEST, 0, MAX_DUTY_CYCLE);
	
	// Need to update below logic else, rampup will be very long.
	if (val > dutyCycle)
	{
		dutyCycle++;
	}
	if (val < dutyCycle)
	{
		dutyCycle--;
	}
}

long adcMap(long x, long inMin, long inMax, long outMin, long outMax)
{
	return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
