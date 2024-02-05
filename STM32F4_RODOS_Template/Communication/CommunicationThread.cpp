#include "CommunicationThread.hpp"

HAL_GPIO ledgreen(GPIO_060);

void CommunicationThread::init()
{
	ledgreen.init(true, 1, 0);
}

void CommunicationThread::run()
{
	// Wait for Electrical
	while (getMode() == Electrical_Startup) suspendCallerUntil(NOW() + 200 * MILLISECONDS);

	// Config
	using namespace config;
	{
		this->period = com_thread_period;
		if (!com_thread_enable) suspendCallerUntil(END_OF_TIME);
	}

	while (true)
	{
		telecommand.processNewCommand();
		telemetry.send_Continuous();

		ledgreen.setPins(~ledgreen.readPins());
		suspendCallerUntil(NOW() + this->period * MILLISECONDS);
	}
}


CommunicationThread communicationthread;