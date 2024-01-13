#ifndef FLOATSAT_DOCKING_DOCKINGTHREAD_HPP_
#define FLOATSAT_DOCKING_DOCKINGTHREAD_HPP_

#include "rodos.h"

#include "ArmControl.hpp"
#include "StepperMoorThread.hpp"

#include "../Threads/Modes.hpp"

class DockingThread : public Thread
{

public:

	// Set name, prio and stack size
	DockingThread() : Thread("Docking Thread", 100, 20000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern DockingThread dockingThread;

#endif 
