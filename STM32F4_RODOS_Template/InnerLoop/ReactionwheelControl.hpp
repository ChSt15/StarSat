#ifndef FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_
#define FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_

#include "rodos.h"
#include "../PIDController.hpp"
#include "../timestamp.hpp"


class ReactionwheelControl: public PID
{
public:

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
