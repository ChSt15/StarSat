#ifndef FLOATSAT_COMMUNICATION_TELECOMANDLIST_HPP_
#define FLOATSAT_COMMUNICATION_TELECOMANDLIST_HPP_

#define TelecommandTopicId 40

// @brief Structure of a Telecommand
struct Command
{
	int id;
	float fval_1, fval_2, fval_3;
	int ival_1, ival_2, ival_3;
};

// @brief List of all telecommandID
enum CommandIds
{
	// Example:
	// Changes mode to modeID given in ival_1
	ChangeMode = 1
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
};

#endif
