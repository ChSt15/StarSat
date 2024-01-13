#ifndef FLOATSAT_INNERLOOP_INNERLOOPTHREAD_HPP_
#define FLOATSAT_INNERLOOP_INNERLOOPTHREAD_HPP_

#include "rodos.h"

#include "Hbridge.hpp"
#include "ReactionwheelControl.hpp"
#include "ReactionwheelEncoder.hpp"
#include "InnerLoopTopics.hpp"

#include "../Threads/Modes.hpp"


class InnerLoopThread : public Thread
{

public:

	// Set name, prio and stack size
	InnerLoopThread() : Thread("Inner Loop Thread", 120, 20000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern InnerLoopThread innerLoopThread;

#endif 