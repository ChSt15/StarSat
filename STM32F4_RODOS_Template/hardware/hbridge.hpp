#ifndef FLOATSAT_HARDWARE_HBRIDGE_HPP
#define FLOATSAT_HARDWARE_HBRIDGE_HPP

#include "rodos.h"


class HBridge
{
private:

	HAL_PWM pwm1;			// positive direction
	HAL_PWM pwm2;			// negative direction

	/**
	 * @note Following parameters have to be chosen so that timerclock/frequency/increments = integer
	 * Possible values: 168Mhz/2000Hz/4000 = 21; 168MHz/5000Hz/1600 = 21; 168MHz/8000Hz/1000 = 21; 168MHz/20kHz/400 = 21;
	 * 					 84Mhz/2000Hz/2000 = 21;   84MHz/5000Hz/800 = 21;   84MHz/8000Hz/500 = 21;   84MHz/20kHz/200 = 21;
	 * PWM_IDX00 - PWM_IDX03: 168 Mhz; all other 84 Mhz
	 * @note For high frequencies (20kHz), we might need to modell non-linearities with sqrt(voltagePercentage) in setVoltage()
	*/
	int pwmFrequency;
	int pwmIncrements;

	// @brief Maximum output voltage percentage of HBridge output.
	float MAX_OUTPUT_PERCENTAGE = 1.0f;
	
	/**
	 * @brief Check if desired percentage of max. voltage exceeds limits and if so, adjust
	 * @param voltagePercentage 
	 * @return adjusted value
	*/
	float checkVoltagePercentage(float voltagePercentage);

public:

	/**
	 * @brief Initialize HBridge -> calls PWM.init() functions
	*/
    void initialization(int pwmFrequency, int pwmIncrements);

	/**
	 * @brief Construct a new HBridge object
	 * @param pwm1 PWM output connected to the HBridge pin 1
	 * @param pwm2 PWM output connected to the HBridge pin 2
	*/
	HBridge(RODOS::PWM_IDX pwm1, RODOS::PWM_IDX pwm2);

	/**
	 * @brief Set desired output voltage of HBridge
	 * @param voltage_percentage Percentage of max.possible output voltage of the HBridge, between -1 and 1. -1 is 100% negative, 1 is 100% positive
	*/
	void setVoltage(float voltagePercentage);

};


extern HBridge hbridge;

#endif