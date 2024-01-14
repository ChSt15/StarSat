#ifndef FLOATSAT_OUTERLOOP_ANGULARVELOCITYCONTROL_HPP_
#define FLOATSAT_OUTERLOOP_ANGULARVELOCITYCONTROL_HPP_

#include "rodos.h"
#include "../PIDController.hpp"
#include "AttitudeEstimation.hpp"

#include "../timestamp.hpp"


class AngularVelocityControl: public PID
{
public:
	/**
	 * @brief Determine output of satellite velocity controller / input of reactionwheel controller
	 * @param attitude_measured: measurement/estimationn of current attitude estimated by Kalman filter
	 * @return Desired speed of reactionwheel that needs to be reached; range of -maxSpeed to +maxSpeed
	*/
	float update(TimestampedData<Attitude_Data> attitude_measured);

};


extern AngularVelocityControl velocitycontrol;

#endif
