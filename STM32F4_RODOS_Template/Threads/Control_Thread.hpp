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
#include "Config.hpp"


class ControlThread : public Thread
{
public:

	// Set name, prio and stack size
	ControlThread() : Thread("Control Thread", 100, 2000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern ControlThread controlthread;

#endif 
