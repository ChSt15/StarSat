#ifndef FLOATSAT_CONTROL_ANGULARVELOCITYCONTROL_HPP_
#define FLOATSAT_CONTROL_ANGULARVELOCITYCONTROL_HPP_

#include "rodos.h"

struct VelocityControlParams
{
	float P = 0;
	float I = 0;
	float D = 0;
	float limit = 12000; /// @brief limit for output in range -limit to limit in RPM
};


class AngularVelocityControl
{
private:

	float desiredAngluarVelocity;

	VelocityControlParams controlParams;

public:

	AngularVelocityControl();

	// @brief Sets Controlparameters
	// @params Controlparameter defined in AngularVelocityControl.hpp
	void setParams(const VelocityControlParams& params);

	// @brief Gets Controlparameters
	// @return Controlparameter defined in AngularVelocityControl.hpp
	const VelocityControlParams& getParams();

	// @brief Sets desired angularvelocity of satellite (Controler Input)
	// @param w_set -> disired angularvelocity [rad/s]
	void setDesiredAngularVelocity(float w_set);

	// @brief Gets the Speed for reactionwheel (Controler Output)
	// @param w_mes -> measured angularvelocity of satellite [rad/s]
	// @return reactionwheel speed [RPM]
	float getSpeed(float w_mes);

};


extern AngularVelocityControl velocitycontrol;

#endif