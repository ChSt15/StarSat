#ifndef FLOATSAT_CONTROL_ANGULARVELOCITYCONTROL_HPP_
#define FLOATSAT_CONTROL_ANGULARVELOCITYCONTROL_HPP_

#include "rodos.h"
#include "PIDController.hpp"
#include "AttitudeEstimation.hpp"

#include "../timestamp.hpp"


class AngularVelocityControl
{
private:

	PID controller;
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
	void setParams(PIDParams params);


	/**
	 * @brief Get control parameters of PID controller
	*/
	PIDParams getParams();


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
	 * @brief Get max. angular velocity that can be reached by satellite [rad/s]
	*/
	float getMaxDesiredVelocity();


	/**
	 * @brief Determine output of satellite velocity controller / input of reactionwheel controller
	 * @param attitude_measured: measurement/estimationn of current attitude estimated by Kalman filter
	 * @return Desired speed of reactionwheel that needs to be reached; range of -maxSpeed to +maxSpeed
	*/
	float update(TimestampedData<Attitude_Data> attitude_measured);

};


extern AngularVelocityControl velocitycontrol;

#endif
