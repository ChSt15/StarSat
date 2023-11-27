#ifndef FLOATSAT_HARDWARE_ELECTRICALMONITORING_HPP_
#define FLOATSAT_HARDWARE_ELECTRICALMONITORING_HPP_

#include "rodos.h"


/**
 * @brief This class monitors the electrical system of the satellite and does some protection logic.
 * 
*/
class ElectricalMonitoring
{
private:

	float batteryVoltage;
	float auxCurrent;
	float reactionWheelCurrent;

	RODOS::GPIO_PIN extPowerPin_;
	RODOS::ADC_CHANNEL adcVBATPin_;
	RODOS::ADC_CHANNEL adcShuntPin_;
	RODOS::ADC_CHANNEL adcWheelPin_;
	RODOS::PWM_IDX beeperIDX_;

	RODOS::HAL_GPIO extPower;
	RODOS::HAL_ADC adcVBAT;
	RODOS::HAL_ADC adcShunt;
	RODOS::HAL_ADC adcWheel;
	RODOS::HAL_PWM beeper;

public:

	/**
	 * @brief Constructs the electrical monitoring object.
	 * @param powerOff   GPIO connected to the external power off switch
	 * @param adcVBAT  ADC channel connected to the battery voltage directly before the shunt resistor
	 * @param adcShunt ADC channel connected directly after the shunt resistor
	 * @param adcWheel ADC channel connected to the reaction wheel driver current output
	*/
	ElectricalMonitoring(RODOS::GPIO_PIN powerOffPin, RODOS::ADC_CHANNEL adcVBATPin, RODOS::ADC_CHANNEL adcShuntPin, RODOS::ADC_CHANNEL adcWheelPin, RODOS::PWM_IDX beeperPin);

	/**
	 * Call once quickly after system startup.
	*/
	void initialize();

	/**
	 * @brief This function is to be called at 10 Hz.
	 * 
	*/
	void update();

	/**
	 * @returns the battery voltage in Volts
	*/
	float getBatteryVoltage();

	/**
	 * @returns the current drawn by everything except the reaction wheels in Ampere
	*/
	float getAuxCurrent();

	/**
	 * @returns the current drawn by the reaction wheels in Ampere.
	*/
	float getReactionWheelCurrent();

	/**
	 * @brief Will switch off external power switch.
	 * @note This function does not return! But preemtive multitasking may cause other tasks to continue running until system is powered off.
	 * 
	*/
	void powerOff();


private:

};


extern ElectricalMonitoring electricalMonitor;

#endif