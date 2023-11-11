#ifndef FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_
#define FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_

#include "rodos.h"

struct ReactionwheelControlParams
{
	float P = 0;
	float I = 0;
	float D = 0;
	float limit = 1; /// @brief limit for output in range -limit to limit
};


class ReactionwheelControl
{
private:

	float desiredSpeed;

	ReactionwheelControlParams controlParams;

public:

	ReactionwheelControl();

	// @brief Sets Controlparameters
	// @params Controlparameter defined in ReactionwheelControl.hpp
	void setParams(const ReactionwheelControlParams& params);

	// @brief Gets Controlparameters
	// @return Controlparameter defined in ReactionwheelControl.hpp
	const ReactionwheelControlParams& getParams();

	// @brief Sets desired Speed for reactionwheel (Controler Input)
	// @param w_set -> disired reactionwheel speed [RPM]
	void setDesiredSpeed(float w_set);

	// @brief Calculates the Power output for reactionwheel (Controller Output)
	// @param rpmMeas -> measured reactionwheel speed [RPM]
	// @return output for power in range of -1 to 1
	float update(float w_mes);

};


extern ReactionwheelControl reactionwheelControl;

#endif