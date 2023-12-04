#include "Telecomand.hpp"


// Telecomand topic
Topic<Command> telecommandTopic(TelecommandTopicId, "Telecomand Topic");

// Telecomand topic subscriber setup
static CommBuffer<Command> commandBuffer;
static Subscriber telecommandSubsciber(telecommandTopic, commandBuffer);
Command commandReceiver;

Topic<float> telecommandtestTopic(80, "Telecomand Test Topic");
static CommBuffer<float> TestcommandBuffer;
static Subscriber testtelecommandSubsciber(telecommandtestTopic, TestcommandBuffer);
float  testcommandReceiver;


void Telecommand::processNewCommand()
{


TestcommandBuffer.get(testcommandReceiver);
telemetry.send_CalibIMU(testcommandReceiver);


	// Only get new Command
	if (commandBuffer.getOnlyIfNewData(commandReceiver))
	{
		switch ((CommandIds) commandReceiver.id)
		{

		default:
			return;
		}

		this->commandCnt++;
	}
}

int Telecommand::getCommandCounter()
{
	return this->commandCnt;
}


Telecommand telecommand;
