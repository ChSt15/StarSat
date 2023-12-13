#ifndef FLOATSAT_CONTROL_ARMCONTROLLER_HPP_
#define FLOATSAT_CONTROL_ARMCONTROLLER_HPP_

#include "rodos.h"
#include "matlib.h
#include <math.h>
#include "../threadsafe.hpp"
#include "../Threads/StepperMotor_Thread.hpp"
#include "../Communication/Camera.hpp"
#include "../Threads/Control_Thread.hpp"


class ArmController
{
private:
	 
	Threadsafe<float> w_Mockup;
	float distance;

	const max_vel = 50;			// [step/s]
	const max_accel = 5;		// [step/s^2]

public:
	bool InitialExtension();

	bool FinalExtension();

	bool CalcAngularVelocity();

	float getAngularvelocityMockup();
};


#endif;