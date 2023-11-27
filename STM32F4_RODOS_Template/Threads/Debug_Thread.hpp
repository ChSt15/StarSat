#ifndef FLOATSAT_THREADS_DEBUGTHREAD_HPP_
#define FLOATSAT_THREADS_DEBUGTHREAD_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"

enum modes
{
	Idle = 0,
	Calib_Gyro, Calib_Accel, Calib_Mag,
	Control_Pos, Control_Vel,
	Mission_Locate, Mission_Point, Mission_Dock
};


// WARNING: just a temporary solution, will be deleted once TTC is implemented
class DebugThread : public Thread
{

public:

	// Set name, prio and stack size
	DebugThread() : Thread("Debug Thread", 100, 2000) {}

	void init();
	void run();

private:

	const int period = 100;		// [ms]
};


extern DebugThread debugthread;
extern Semaphore mode_protec;
extern modes mode;

#endif 