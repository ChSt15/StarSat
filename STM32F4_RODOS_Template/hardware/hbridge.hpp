#ifndef FLOATSAT_HARDWARE_HBRIDGE_HPP
#define FLOATSAT_HARDWARE_HBRIDGE_HPP

#include "rodos.h"


class HBridge
{
private:

	HAL_PWM pwm1;
	HAL_PWM pwm2;

	// i think these were recommended, change if i am wrong - max; 
	// Chris: We might want to test 20Khz, the non linearities could be roughly modelled as actual = input^2, so correcting with output = sqrt(input) could roughly
	// correct for the non linearities. As long as all values remain between 0 and 1. For negative values we could just invert the sign of the output.
	int pwmFrequency = 5000;
	int pwmIncrements = 1000;

public:

	/**
	 * @brief Construct a new HBridge object
	 * @param pwm1 PWM output connected to the HBridge pin 1
	 * @param pwm2 PWM output connected to the HBridge pin 2
	*/
	HBridge(RODOS::PWM_IDX pwm1, RODOS::PWM_IDX pwm2);

	/**
	 * @brief Set the power of the HBridge
	 * @param power Power of the HBridge, between -1 and 1. -1 is 100% negative, 1 is 100% positive
	 * @note Actual power is dependent on the voltage supplied to the HBridge
	*/
	void setPower(float power);

};


extern HBridge hbridge;

#endif