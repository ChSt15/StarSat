#ifndef FLOATSAT_HARDWARE_REACTIONWHEELENCODER_HPP_
#define FLOATSAT_HARDWARE_REACTIONWHEELENCODER_HPP_

#include "rodos.h"


// i think we will get a template for this, so this is just a minimal overview
class ReactionwheelEncoder
{

public:

	ReactionwheelEncoder();

	// @brief Gets speed of reactionwheel
	// @return reactionwheel speed [RPM]
	float getSpeed();

private:

};


extern ReactionwheelEncoder encoder;

#endif