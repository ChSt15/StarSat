#ifndef FLOATSAT_HARDWARE_ADC_HPP
#define FLOATSAT_HARDWARE_ADC_HPP

#include "rodos.h"

class ADC
{
private:

	HAL_ADC adc;

	const int resolution = 4095;				// 12 bits -> 2^12 - 1 = 4095
    const float referenceVoltage = 3000.0;		// mV

    RODOS::ADC_CHANNEL adc_channel;

public:

	/**
	 * @brief Initialize ADC -> calls adc.init() function
	*/
    void initialization();

	/**
	 * @brief Construct a new HBridge object
	 * @param adc_idx ADC pin
     * @param adc_channel ADC channel
	*/
	ADC(RODOS::ADC_IDX adc_idx, RODOS::ADC_CHANNEL adc_channel);

	/**
	 * @brief Get read input voltage of ADC
	*/
	float getVoltage();

};


extern ADC adc;

#endif