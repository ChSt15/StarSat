#ifndef FLOATSAT_COMMUNICATION_TELECOMAND_HPP_
#define FLOATSAT_COMMUNICATION_TELECOMAND_HPP_

#include "rodos.h"

#include "Gateway.hpp"


class Telecomand
{
private:

	TopicListReport topics;

public:

	Telecomand();

	void process();
};


extern Telecomand telecomand;

#endif
