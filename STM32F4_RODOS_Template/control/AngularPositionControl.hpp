#ifndef FLOATSAT_CONTROL_ANGULARPOSITIONCONTROL_HPP_
#define FLOATSAT_CONTROL_ANGULARPOSITIONCONTROL_HPP_

#include "rodos.h"

struct PositionControlParams
{
	float P;
	float I;
	float D;
};


class AngularPositionControl
{

public:

	AngularPositionControl();

	// @brief Sets Controlparameters
	// @params Controlparameter defined in AngularPositionControl.hpp
	void setParams(PositionControlParams params);

	// @brief Gets Controlparameters
	// @return Controlparameter defined in AngularPositionControl.hpp
	PositionControlParams getParams();

	// @brief Sets disired angle of satellite (Controler Input)
	// @param angle_set -> disired angle [rad]
	void setDisiredAngle(float angle_set);

	// @brief Gets the Speed for reactionwheel (Controler Output)
	// @param angle_mes -> measured angle of satellite [rad]
	// @return reactionwheel speed [RPM]
	float getSpeed(float angle_mes);

private:

	float disiredAngle;

	PositionControlParams controlParams;
};


extern AngularPositionControl positionControl;

#endif