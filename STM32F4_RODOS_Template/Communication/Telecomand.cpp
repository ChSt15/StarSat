#include "Telecomand.hpp"


// Telecomand topic
Topic<Command> telecommandTopic(TelecommandTopicId, "Telecomand Topic");

// Telecomand topic subscriber setup
static CommBuffer<Command> commandBuffer;
static Subscriber telecommandSubsciber(telecommandTopic, commandBuffer);
Command commandReceiver;


void Telecommand::processNewCommand()
{
	// Only get new Command
	if (commandBuffer.getOnlyIfNewData(commandReceiver))
	{
		switch ((CommandIds) commandReceiver.id)
		{
		case ChangeMode:
			break;

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