#include "Communication_Thread.hpp"


void CommunicationThread::init()
{

}

void CommunicationThread::run()
{
	while (true)
	{
		
		telecommand.processNewCommand();

		telemetry.send_Continuous();
	
		suspendCallerUntil(NOW() + this->period * MILLISECONDS);
	}
}


//CommunicationThread communicationthread;
