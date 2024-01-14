#ifndef FLOATSAT_OUTERLOOP_ANGULARPOSITIONCONTROL_HPP_
#define FLOATSAT_OUTERLOOP_ANGULARPOSITIONCONTROL_HPP_

#include "rodos.h"
#include "../PIDController.hpp"
#include "AttitudeEstimation.hpp"

#include "../timestamp.hpp"


class AngularPositionControl: public PID
{
public:

	/**
	 * @brief Set desired yaw angle/orientation of satellite in [rad]
	*/
	void setSetpoint(float angle_set);

	/**
	 * @brief Determine output of angular position controller / input of angular velocity controller
	 * @param attitude_measured: measurement/estimationn of current attitude estimated by Kalman filter
	 * @return Desired angular velocity of satellite that needs to be reached; range of -maxAngularVelocity to +maxAngularVelocity
	*/
	float update(TimestampedData<Attitude_Data> attitude_measured);

};


extern AngularPositionControl positionControl;

extern Topic<float> AngularPositionSetpointTopic;

#endif
