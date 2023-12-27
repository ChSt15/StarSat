#ifndef FLOATSAT_THREADS_DEBUGTHREAD_HPP_
#define FLOATSAT_THREADS_DEBUGTHREAD_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"
#include "Config.hpp"


// WARNING: just a temporary solution, will be deleted once TTC is implemented
class DebugThread : public Thread
{

public:

	// Set name, prio and stack size
	DebugThread() : Thread("Debug Thread", 100, 2000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern DebugThread debugthread;

#endif 