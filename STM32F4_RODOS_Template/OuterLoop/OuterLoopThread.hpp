#ifndef FLOATSAT_OUTER_OUTERLOOPTHREAD_HPP_
#define FLOATSAT_OUTER_OUTERLOOPTHREAD_HPP_

#include "rodos.h"

#include "AngularPositionControl.hpp"
#include "AngularVelocityControl.hpp"
#include "AttitudeEstimation.hpp"
#include "IMU.hpp"
#include "IMUCalibration.hpp"

#include "../Threads/Modes.hpp"
#include "../InnerLoop/InnerLoopTopics.hpp"
#include "../InnerLoop/InnerLoopThread.hpp"

class OuterLoopThread : public Thread
{

public:

	// Set name, prio and stack size
	OuterLoopThread() : Thread("Outer Loop Thread", 120, 2000) {}

	void init();
	void run();

private:

	int period;		// [ms]

	void publishSpeed(float speed);
};


extern OuterLoopThread outerLoopThread;

#endif 