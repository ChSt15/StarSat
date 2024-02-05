#ifndef FLOATSAT_THREADS_CONTROLTHREAD_HPP_
#define FLOATSAT_THREADS_CONTROLTHREAD_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"
#include "../InnerLoop/Hbridge.hpp"
#include "Adc.hpp"
#include "../Config.hpp"


class ControlThread : public Thread
{
public:

	// Set name, prio and stack size
	ControlThread() : Thread("Control Thread", 100, 1000) {}

	void init();
	void run();

private:

	int period = 100;		// [ms]
};


extern ControlThread controlthread;

#endif 
