#ifndef FLOATSAT_COMMUNICATION_TELECOMAND_HPP_
#define FLOATSAT_COMMUNICATION_TELECOMAND_HPP_

#include "rodos.h"

#include "TelecommandList.hpp"
#include "Telemetry.hpp"
#include "../Modes.hpp"
#include "../PIDController.hpp"
#include "../OuterLoop/IMU.hpp"
#include "../OuterLoop/AngularPositionControl.hpp"
#include "../OuterLoop/AngularVelocityControl.hpp"
#include "../InnerLoop/ReactionwheelControl.hpp"


class Telecommand
{
private:

	int commandCnt = 0;
	int lastCmndID = 0;

public:

	// @brief Processes the last new command received (if there is one)
	void processNewCommand();

	// @brief Returns number of successfully received commands (not thread safe)
	int getCommandCounter();

	int getLastCommand();
};

// @brief Global telecommand object
extern Telecommand telecommand;

// @brief Global telecommand topic
extern Topic<Command> telecommandTopic;

extern Topic<Command> EchoTopic;

#endif
