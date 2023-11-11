#ifndef FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_
#define FLOATSAT_CONTROL_REACTIONWHEELCONTROL_HPP_

#include "rodos.h"

struct ReactionwheelControlParams
{
	float P;
	float I;
};


class ReactionwheelControl
{

public:

	ReactionwheelControl();

	// @brief Sets Controlparameters
	// @params Controlparameter defined in ReactionwheelControl.hpp
	void setParams(ReactionwheelControlParams params);

	// @brief Gets Controlparameters
	// @return Controlparameter defined in ReactionwheelControl.hpp
	ReactionwheelControlParams getParams();

	// @brief Sets disired Speed for reactionwheel (Controler Input)
	// @param v_set -> disired reactionwheel speed [RPM]
	void setDisiredSpeed(float v_set);

	// @brief Gets the Voltage for reactionwheel (Controler Output)
	// @param v_mes -> measured reactionwheel speed [RPM]
	// @return voltage in V 
	float getVoltage(float v_mes);

private:

	float disiredSpeed;

	ReactionwheelControlParams controlParams;
};


extern ReactionwheelControl reacwheelcontrol;

#endif