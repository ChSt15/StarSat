#ifndef FLOATSAT_HARDWARE_HBRIDGE_HPP_
#define FLOATSAT_HARDWARE_HBRIDGE_HPP_

#include "rodos.h"


class HBridge
{

public:

	HBridge();

	// @brief Sets voltage of HBridge for reactionwheel
	// @param voltage -> voltage in V (in range of -5V and +5V)
	void setVoltage(float voltage);

private:

	// compiler complains if defined without decleration
	//HAL_PWM pwm1;
	//HAL_PWM pwm2;

	// i think these were recommended, change if i am wrong - max
	int pwm_frequency = 5000;
	int pwm_increments = 1000;
};


extern HBridge hbridge;

#endif