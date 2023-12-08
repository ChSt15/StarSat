#ifndef FLOATSAT_THREADS_CONTROLTHREAD_HPP_
#define FLOATSAT_THREADS_CONTROLTHREAD_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"

#include "../control/AttitudeEstimation.hpp"
#include "../control/PIDController.hpp"
#include "../control/ReactionwheelControl.hpp"
#include "../control/AngularVelocityControl.hpp"
#include "../control/AngularPositionControl.hpp"

#include "../hardware/ReactionwheelEncoder.hpp"
#include "../hardware/hbridge.hpp"

#include "Debug_Thread.hpp"
#include "Modes.hpp"


/**
 * @todo NEEDS TO BE SPECIFIED
*/
PIDParams paramsReactionWheelControl {1.0f, 1.0f, 1.0f};
PIDParams paramsVelocityControl {1.0f, 1.0f, 1.0f};
PIDParams paramsPositionControl {1.0f, 1.0f, 1.0f};
float maxVoltage = 10.0f;									// [V]
float maxSpeed = (10000.0f * 2*M_PI) / 60.0f;				// [rad/s]
float maxVelocity = M_PI_2;  								// [rad/s]


class ControlThread : public Thread
{

public:

	// Set name, prio and stack size
	ControlThread() : Thread("Control Thread", 100, 2000) {}

	void init();
	void run();

private:

	const int period = 100;		// [ms]
};


extern ControlThread controlthread;

#endif 
