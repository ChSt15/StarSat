#include "Communication_Thread.hpp"


void CommunicationThread::init()
{

}

void CommunicationThread::run()
{
	while (true)
	{
		
		telecomand.process();

		telemetry.send_Continuous();
	
		suspendCallerUntil(NOW() + this->period * MILLISECONDS);
	}
}


CommunicationThread communicationthread;