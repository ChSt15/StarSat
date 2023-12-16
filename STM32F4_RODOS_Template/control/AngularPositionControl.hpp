#ifndef FLOATSAT_CONTROL_ANGULARPOSITIONCONTROL_HPP_
#define FLOATSAT_CONTROL_ANGULARPOSITIONCONTROL_HPP_

#include "rodos.h"
#include "PIDController.hpp"
#include "AttitudeEstimation.hpp"

#include "../timestamp.hpp"


class AngularPositionControl
{
private:

	PID controller;
	float maxDesiredAngle = 2 * M_PI;			// max. angle the satellite can reach in both directions -> to limit the setpoint [rad]

public:

	AngularPositionControl();

	/**
	 * @brief Initialize PID controller
	*/
	void init(const PIDParams& params, float maxAngularVelocity);


	/**
	 * @brief Set control parameters of PID controller
	*/
	void setParams(PIDParams params);


	/**
	 * @brief Get control parameters of PID controller
	*/
	PIDParams getParams();


	/**
	 * @brief Set desired yaw angle/orientation of satellite in [rad]
	*/
	void setDesiredAngle(float angle_set);


	/**
	 * @brief Set max. angular velocity that can be reached by the satellite [rad/s]
	*/
	void setMaxAngularVelocity(float maxAngularVelocity);


	/**
	 * @brief Get max. angular velocity that can be reached by the satellite [rad/s]
	*/
	float getMaxAngularVelocity();


	/**
	 * @brief Determine output of angular position controller / input of angular velocity controller
	 * @param attitude_measured: measurement/estimationn of current attitude estimated by Kalman filter
	 * @return Desired angular velocity of satellite that needs to be reached; range of -maxAngularVelocity to +maxAngularVelocity
	*/
	float update(TimestampedData<Attitude_Data> attitude_measured);

};


extern AngularPositionControl positionControl;

#endif
