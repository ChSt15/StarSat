#ifndef FLOATSAT_INNERLOOP_INNERLOOPTHREAD_HPP_
#define FLOATSAT_INNERLOOP_INNERLOOPTHREAD_HPP_

#include "rodos.h"

#include "Hbridge.hpp"
#include "ReactionwheelControl.hpp"
#include "ReactionwheelEncoder.hpp"
#include "InnerLoopTopics.hpp"

#include "../Modes.hpp"
#include "../Config.hpp"


class InnerLoopThread : public Thread
{

public:

	// Set name, prio and stack size
	InnerLoopThread() : Thread("Inner Loop Thread", 130, 1000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern InnerLoopThread innerLoopThread;

extern Topic<float> mockupSpeedSetpointTopic;

#endif 
