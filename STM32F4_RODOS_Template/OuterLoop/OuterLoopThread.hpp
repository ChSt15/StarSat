#ifndef FLOATSAT_OUTERLOOP_OUTERLOOPTHREAD_HPP_
#define FLOATSAT_OUTERLOOP_OUTERLOOPTHREAD_HPP_

#include "rodos.h"

#include "AngularPositionControl.hpp"
#include "AngularVelocityControl.hpp"
#include "AttitudeEstimation.hpp"
#include "IMU.hpp"
#include "IMUCalibration.hpp"

#include "../Modes.hpp"
#include "../Config.hpp"
#include "../InnerLoop/InnerLoopTopics.hpp"
#include "../InnerLoop/InnerLoopThread.hpp"
#include "../Communication/Camera.hpp"
#include "../Communication/Telemetry.hpp"


class OuterLoopThread : public Thread
{

public:

	// Set name, prio and stack size
	OuterLoopThread() : Thread("Outer Loop Thread", 120, 10000) {}

	void init();
	void run();

private:

	int period;		// [ms]

	void publishSpeed(float speed);
};


extern OuterLoopThread outerLoopThread;

#endif 
