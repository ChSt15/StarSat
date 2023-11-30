#ifndef FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_
#define FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_

#include "rodos.h"
#include "PIDController.hpp"
#include "timestamp.hpp"


class ReactionwheelControl
{
private:

	// TimestampedData<float> Speed;
	PID controller;
	float maxVoltage;

public:

	ReactionwheelControl();

	/**
	 * @brief Initialize PID controller
	*/
	void init(const PIDParams& params, float maxLimit, float minLimit, float maxVoltage);


	/**
	 * @brief Set control parameters of PID controller
	*/
	void setParams(const PIDParams& params);


	/**
	 * @brief Get control parameters of PID controller
	*/
	const PIDParams& getParams();


	/**
	 * @brief Set desired speed for reaction wheel in [rad/s]
	*/
	void setDesiredSpeed(float w_set);


	/**
	 * @brief Set max. voltage that can be applied by HBridge [V]
	*/
	void setMaxVoltage(float maxVoltage);


	/**
	 * @brief Get max. voltage that can be applied by HBridge [V]
	*/
	float getMaxVoltage();


	/**
	 * @brief Determine output of reactionwheel controller / input of HBridge
	 * @param speed: measurement of current speed of reaction wheel measured by Encoder in [rad/s]
	 * @return Percentage of max. voltage that needs to be applied by HBridge; range of -1 to 1
	*/
	float update(TimestampedData<float> speed);

};


extern ReactionwheelControl reactionwheelControl;

#endif