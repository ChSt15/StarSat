#ifndef FLOATSAT_HARDWARE_HBRIDGE_HPP_
#define FLOATSAT_HARDWARE_HBRIDGE_HPP_

#include "rodos.h"

class HBridge
{

public:

	HBridge(int frequency, int increments, PWM_IDX pwm1_id, PWM_IDX pwm2_id);
	void setVoltage(float voltage);

private:

	HAL_PWM pwm1;
	HAL_PWM pwm2;

	int pwm_frequency;
	int pwm_increments;
};

extern HBridge hbridge;

#endif