#ifndef FLOATSAT_HARDWARE_REACTIONWHEELENCODER_HPP_
#define FLOATSAT_HARDWARE_REACTIONWHEELENCODER_HPP_

#include "rodos.h"

/**
 * @notes!!!!!!!!!!!!!!!!!1 
 * - Check the class consts. They are only temporary values!
*/

class ReactionwheelEncoder
{

public:

	ReactionwheelEncoder();

	void Init();

	// @brief Gets speed of reactionwheel
	// @return reactionwheel speed RAD/s
	float getSpeed();

private:

};


extern ReactionwheelEncoder encoder;

#endif