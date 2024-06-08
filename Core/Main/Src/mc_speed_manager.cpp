#include "mc_main.hpp"

int16_t mapToReference(int16_t x, int16_t inMin, int16_t inMax, int16_t outMin, int16_t outMax);

void speedManager()
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

	if ((sravg > REQUEST_CCW) && (sravg < REQUEST_CW))
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
			txData.direction = Rotate_CCW;
		}
		if (sravg >= REQUEST_CW)
		{
			val = mapToReference(sravg, REQUEST_CW, MAX_ADC_COUNT, MIN_RPM, MAX_RPM);
			txData.direction = Rotate_CW;
		}
		pidController->setReference(val);
	}
	else
	{
		if (sravg <= REQUEST_CCW)
		{
			val = mapToReference(sravg, 0, REQUEST_CCW, MIN_PWM_PULSE, MAX_PWM_PULSE);
			txData.direction = Rotate_CCW;
		}
		if (sravg >= REQUEST_CW)
		{
			val = mapToReference(sravg, REQUEST_CW, MAX_ADC_COUNT, MIN_PWM_PULSE, MAX_PWM_PULSE);
			txData.direction = Rotate_CW;
		}

		if (val > txData.output_pulse)
		{
			txData.output_pulse += RAMP_UP_FACTOR % (val - txData.output_pulse);
		}
		if (val < txData.output_pulse)
		{
			txData.output_pulse -= RAMP_UP_FACTOR % (txData.output_pulse - val);
		}
	}
}

void pidManager()
{
	if ((PIDStatus_DISABLED == settings.pidStatus) ||
		(!flags.startupCompleteFlag) || (!flags.tmrPidFlag))
	{
		return;
	}
	if ((--timers.pidTimer) == 0)
	{
		timers.pidTimer = TIMEBASE_PID_COUNT;
		txData.output_pulse = pidController->updateOutput(txData.speed_fdbk);
	}
}

int16_t mapToReference(int16_t x, int16_t inMin, int16_t inMax, int16_t outMin, int16_t outMax)
{
	return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
