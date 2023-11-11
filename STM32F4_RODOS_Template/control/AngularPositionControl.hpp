#ifndef FLOATSAT_CONTROL_ANGULARPOSITIONCONTROL_HPP_
#define FLOATSAT_CONTROL_ANGULARPOSITIONCONTROL_HPP_

#include "rodos.h"

struct PositionControlParams
{
	float P = 0;
	float I = 0;
	float D = 0;
	float limit = 2*3.14; /// @brief limit for output in range -limit to limit in rad
};


class AngularPositionControl
{
private:

	float desiredAngle;

	PositionControlParams controlParams;

public:

	AngularPositionControl();

	// @brief Sets Controlparameters
	// @params Controlparameter defined in AngularPositionControl.hpp
	void setParams(const PositionControlParams& params);

	// @brief Gets Controlparameters
	// @return Controlparameter defined in AngularPositionControl.hpp
	const PositionControlParams& getParams();

	// @brief Sets desired angle of satellite (Controler Input)
	// @param angle_set -> disired angle [rad]
	void setDesiredAngle(float angle_set);

	// @brief Gets the Speed for reactionwheel (Controler Output)
	// @param angle_mes -> measured angle of satellite [rad]
	// @return reactionwheel speed [RPM]
	float getSpeed(float angle_mes);

};


extern AngularPositionControl positionControl;

#endif