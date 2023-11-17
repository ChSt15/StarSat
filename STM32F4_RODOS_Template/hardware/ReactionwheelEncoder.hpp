#ifndef FLOATSAT_HARDWARE_REACTIONWHEELENCODER_HPP_
#define FLOATSAT_HARDWARE_REACTIONWHEELENCODER_HPP_

#include "rodos.h"

/**
 * @notes!!!!!!!!!!!!!!!!!1 
 * - Check the class consts. They are only temporary values!
*/


// i think we will get a template for this, so this is just a minimal overview
class ReactionwheelEncoder
{

public:

	static constexpr float REACTIONWHEEL_ENCODER_RESOLUTION = 1; // Number of counts per revolution

	ReactionwheelEncoder(RODOS::GPIO_PIN inA = RODOS::GPIO_000, RODOS::GPIO_PIN inB = RODOS::GPIO_000);

	// @brief Gets speed of reactionwheel
	// @return reactionwheel speed RAD/s
	float getSpeed();

private:

};


extern ReactionwheelEncoder encoder;

#endif