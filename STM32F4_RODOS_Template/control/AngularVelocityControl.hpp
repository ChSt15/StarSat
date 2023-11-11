#ifndef FLOATSAT_CONTROL_ANGULARVELOCITYCONTROL_HPP_
#define FLOATSAT_CONTROL_ANGULARVELOCITYCONTROL_HPP_

#include "rodos.h"

struct VelocityControlParams
{
	float P;
	float I;
};


class AngularVelocityControl
{

public:

	AngularVelocityControl();

	// @brief Sets Controlparameters
	// @params Controlparameter defined in AngularVelocityControl.hpp
	void setParams(VelocityControlParams params);

	// @brief Gets Controlparameters
	// @return Controlparameter defined in AngularVelocityControl.hpp
	VelocityControlParams getParams();

	// @brief Sets disired angularvelocity of satellite (Controler Input)
	// @param w_set -> disired angularvelocity [rad/s]
	void setDisiredAngularVelocity(float w_set);

	// @brief Gets the Speed for reactionwheel (Controler Output)
	// @param w_mes -> measured angularvelocity of satellite [rad/s]
	// @return reactionwheel speed [RPM]
	float getSpeed(float w_mes);

private:

	float disiredAngluarVelocity;

	VelocityControlParams controlParams;
};


extern AngularVelocityControl velocitycontrol;

#endif