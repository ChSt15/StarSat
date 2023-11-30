#include "Telecomand.hpp"

// Dummy topic + sub
Topic<int> DummyTopic(40, "IMU_Telemetry");
static CommBuffer<int> DummyBuffer;
static Subscriber DummyDataSubsciber(DummyTopic, DummyBuffer);
int DummyReceiver;


Telecomand::Telecomand()
{
	// topics to forward
	topics.add(DummyTopic.topicId);
	//..
	uart_gateway.setTopicsToForward(&(this->topics));
}


void Telecomand::process()
{
	DummyBuffer.get(DummyReceiver);
}



Telecomand telecomand;
