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
	timers.dutyTimer = TIMEBASE_DUTY_COUNT;

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

	switch (mcParams.msg.pid_status)
	{
	case McParams_PidStatus_DISABLED:
		if (sravg <= REQUEST_CCW)
		{
			val = mapToReference(sravg, 0, REQUEST_CCW, MIN_PWM_PULSE, MAX_PWM_PULSE);
			mcData.msg.direction = McData_Rotate_CCW;
		}
		if (sravg >= REQUEST_CW)
		{
			val = mapToReference(sravg, REQUEST_CW, MAX_ADC_COUNT, MIN_PWM_PULSE, MAX_PWM_PULSE);
			mcData.msg.direction = McData_Rotate_CW;
		}

		if (val > mcData.msg.output_pulse)
		{
			mcData.msg.output_pulse += RAMP_UP_FACTOR % (val - mcData.msg.output_pulse);
		}
		if (val < mcData.msg.output_pulse)
		{
			mcData.msg.output_pulse -= RAMP_UP_FACTOR % (mcData.msg.output_pulse - val);
		}
		break;
	case McParams_PidStatus_ENABLED:
		if (sravg <= REQUEST_CCW)
		{
			val = mapToReference(sravg, 0, REQUEST_CCW, MIN_RPM, MAX_RPM);
			mcData.msg.direction = McData_Rotate_CCW;
		}
		if (sravg >= REQUEST_CW)
		{
			val = mapToReference(sravg, REQUEST_CW, MAX_ADC_COUNT, MIN_RPM, MAX_RPM);
			mcData.msg.direction = McData_Rotate_CW;
		}

		pidController->setReference(val);
		break;
	}
}

void pidManager()
{
	if ((McParams_PidStatus_DISABLED == mcParams.msg.pid_status) ||
		(!flags.startupCompleteFlag) || (!flags.tmrPidFlag))
	{
		return;
	}
	if ((--timers.pidTimer) == 0)
	{
		timers.pidTimer = TIMEBASE_PID_COUNT;
		mcData.msg.output_pulse = pidController->updateOutput(mcData.msg.speed_fdbk);
	}
}

int16_t mapToReference(int16_t x, int16_t inMin, int16_t inMax, int16_t outMin, int16_t outMax)
{
	return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
