#ifndef FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_
#define FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_

#include "rodos.h"
#include "PIDController.hpp"
#include "../timestamp.hpp"


class ReactionwheelControl
{
private:

	PID controller;
	// TODO set value
	float maxDesiredSpeed;			// max. speed the reactionwheel can reach in both directions -> to limit the setpoint [rad/s]

public:

	ReactionwheelControl();

	/**
	 * @brief Initialize PID controller
	*/
	void init(const PIDParams& params, float maxVoltage, float maxDesiredSpeed, bool use_BackCalculation, bool use_DerivativofMeasurment);


	/**
	 * @brief Set control parameters of PID controller
	*/
	void setParams(PIDParams params);


	/**
	 * @brief Get control parameters of PID controller
	*/
	PIDParams getParams();


	/**
	 * @brief Set desired speed for reaction wheel in [rad/s]
	*/
	void setDesiredSpeed(float w_set);


	/**
	 * @brief Set max. voltage that can be applied by HBridge [V] -> Limit of PID-controller output
	*/
	void setMaxVoltage(float maxVoltage);


	/**
	 * @brief Get max. voltage that can be applied by HBridge [V] -> Limit of PID-controller output
	*/
	float getMaxVoltage();


	/**
	 * @brief Get max. speed that can the reactionwheel can reach [rad/s]
	*/
	float getMaxDesiredSpeed();


	/**
	 * @brief Determine output of reactionwheel controller / input of HBridge
	 * @param speed_measured: measurement of current speed of reaction wheel measured by Encoder in [rad/s]
	 * @return Percentage of max. voltage that needs to be applied by HBridge; range of -1 to 1
	*/
	float update(TimestampedData<float> speed_measured);

};


extern ReactionwheelControl reactionwheelControl;

#endif
