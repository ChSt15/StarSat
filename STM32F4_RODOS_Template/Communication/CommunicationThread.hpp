#ifndef FLOATSAT_COMMUNICATION_COMMUNICATIONTHREAD_HPP_
#define FLOATSAT_COMMUNICATION_COMMUNICATIONTHREAD_HPP_

#include "rodos.h"

#include "Telemetry.hpp"
#include "Telecomand.hpp"
#include "../Config.hpp"


class CommunicationThread : public Thread
{

public:

	// Set name, prio and stack size
	CommunicationThread() : Thread("Communication Thread", 100, 1000) {}

	void init();
	void run();

private:

	int period;		// [ms]
};


extern CommunicationThread communicationthread;

#endif 
