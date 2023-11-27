#ifndef FLOATSAT_THREADS_CONTROLTHREAD_HPP_
#define FLOATSAT_THREADS_CONTROLTHREAD_HPP_

#include "rodos.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"
#include "../timestamp.hpp"
#include "Debug_Thread.hpp"


class ControlThread : public Thread
{

public:

	// Set name, prio and stack size
	ControlThread() : Thread("Control Thread", 100, 20000) {}

	void init();
	void run();

private:

	const int period = 100;		// [ms]
};


extern ControlThread controlthread;

#endif 
