#include "Communication_Thread.hpp"

HAL_GPIO ledgreen(GPIO_060);

void CommunicationThread::init()
{
	ledgreen.init(true, 1, 0);
}

void CommunicationThread::run()
{
	while (true)
	{
		telecommand.processNewCommand();
		telemetry.send_Continuous();

		ledgreen.setPins(~ledgreen.readPins());
		suspendCallerUntil(NOW() + this->period * MILLISECONDS);
	}
}


CommunicationThread communicationthread;
