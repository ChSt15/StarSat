#include "Communication_Thread.hpp"

HAL_GPIO ledgreen(GPIO_060);

void CommunicationThread::init()
{
	ledgreen.init(true, 1, 0);
}

void CommunicationThread::run()
{
	// Config
	using namespace config;
	{
		this->period = com_thread_period;
		if (!com_thread_enable) suspendCallerUntil(END_OF_TIME);
	}

	while (true)
	{
		telecommand.processNewCommand();
		//telemetry.send_Continuous();

		ledgreen.setPins(~ledgreen.readPins());
		suspendCallerUntil(NOW() + this->period * MILLISECONDS);
	}
}


//CommunicationThread communicationthread;
