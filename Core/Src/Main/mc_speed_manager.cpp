#include "mc_main.hpp"

int16_t mapToReference(int16_t x, int16_t inMin, int16_t inMax, int16_t outMin, int16_t outMax);

// TODO: look for other way to update pid output

void speedManager(void)
{
	static uint32_t srsum = 0;
	static uint32_t sravg = 0;

	if (flags.tmrDutyFlag)
	{
		return;
	}
	flags.tmrDutyFlag = 0;

	if (--timers.dutyTimer)
	{
		return;
	}
	timers.dutyTimer = TIMEBASE_DUTY_RAMP;

	srsum -= sravg;
	srsum += ADC_USR_POT;

	sravg = srsum >> ADC_AVG_FACTOR;

	if (!flags.startupCompleteFlag)
	{
		return;
	}

	if (sravg > REQUEST_CCW && sravg < REQUEST_CW)
	{
		if (flags.runFlag)
		{
			flags.stopFlag = 1;
		}
		flags.runFlag = 0;
		return;
	}
	else
	{
		flags.runFlag = flags.powerFlag;
	}

	uint16_t val = 0;
	if (PIDStatus_ENABLED == settings.pidStatus)
	{
		if (sravg <= REQUEST_CCW)
		{
			val = mapToReference(sravg, 0, REQUEST_CCW, MIN_RPM, MAX_RPM);
			readings->direction = Rotate_CCW;
		}
		if (sravg >= REQUEST_CW)
		{
			val = mapToReference(sravg, REQUEST_CW, MAX_ADC_COUNT, MIN_RPM, MAX_RPM);
			readings->direction = Rotate_CW;
		}
		pidController->setReference(val);
		if (flags.runFlag && (0 == (--timers.pidTimer)))
		{
			readings->outputPulse = pidController->updateOutput(readings->getSpeedFdbk());
			timers.pidTimer = TIMEBASE_PID_STEP;
		}
	}
	else
	{
		if (sravg <= REQUEST_CCW)
		{
			val = mapToReference(sravg, 0, REQUEST_CCW, MIN_PWM_PULSE, MAX_PWM_PULSE);
			readings->direction = Rotate_CCW;
		}
		if (sravg >= REQUEST_CW)
		{
			val = mapToReference(sravg, REQUEST_CW, MAX_ADC_COUNT, MIN_PWM_PULSE, MAX_PWM_PULSE);
			readings->direction = Rotate_CW;
		}

		if (val > readings->outputPulse)
		{
			readings->outputPulse = readings->outputPulse + (RAMP_UP_FACTOR % (val - readings->outputPulse));
		}
		if (val < readings->outputPulse)
		{
			readings->outputPulse = readings->outputPulse - (RAMP_UP_FACTOR % (readings->outputPulse - val));
		}
	}
}

int16_t mapToReference(int16_t x, int16_t inMin, int16_t inMax, int16_t outMin, int16_t outMax)
{
	return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
