#ifndef FLOATSAT_CONTROL_ANGULARVELOCITYCONTROL_HPP_
#define FLOATSAT_CONTROL_ANGULARVELOCITYCONTROL_HPP_

#include "rodos.h"
#include "PIDController.hpp"
#include "timestamp.hpp"


class AngularVelocityControl
{
private:

	PID controller;
	float maxSpeed;						// max. speed the reaction wheel can reach -> to limit the control output [rad/s]
	float maxDesiredVelocity;			// max. angular velocity the satellite can reach in both directions -> to limit the setpoint [rad/s]

public:

	AngularVelocityControl();

	/**
	 * @brief Initialize PID controller
	*/
	void init(const PIDParams& params, float maxSpeed, float maxDesiredVelocity);


	/**
	 * @brief Set control parameters of PID controller
	*/
	void setParams(const PIDParams& params);


	/**
	 * @brief Get control parameters of PID controller
	*/
	const PIDParams& getParams();


	/**
	 * @brief Set desired angular velocity of satellite in [rad/s]
	*/
	void setDesiredAngularVelocity(float w_set);


	/**
	 * @brief Set max. speed that can be reached by reactionwheel [rad/s]
	*/
	void setMaxSpeed(float maxSpeed);


	/**
	 * @brief Get max. speed that can be reached by reactionwheel [rad/s]
	*/
	float getMaxSpeed();


	/**
	 * @brief Set max. angular velocity that can be reached by satellite [rad/s]
	*/
	void setMaxDesiredVelocity(float maxDesiredVelocity);


	/**
	 * @brief Get max. angular velocity that can be reached by satellite [rad/s]
	*/
	float getMaxDesiredVelocity();


	/**
	 * @brief Determine output of satellite velocity controller / input of reactionwheel controller
	 * @param velocity_measured: measurement of current angular velocity (about z-axis) of satellite measured by Gyroscope in [rad/s]
	 * @return Desired speed of reactionwheel that needs to be reached; range of -maxSpeed to +maxSpeed
	*/
	float update(TimestampedData<float> velocity_measured);

};


extern AngularVelocityControl velocitycontrol;

#endif