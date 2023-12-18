#ifndef FLOATSAT_CONTROL_ARMCONTROLLER_HPP_
#define FLOATSAT_CONTROL_ARMCONTROLLER_HPP_

#include "rodos.h"
#include "matlib.h"
#include <math.h>
#include "../Threads/StepperMotor_Thread.hpp"
#include "../Communication/Camera.hpp"


class ArmController
{
private:
	 
	Semaphore sem;

	float w_Mockup;
	float distance;

	const int max_vel = 50;			// [step/s]
	const int min_vel = 1;			// [step/s]
	const int max_accel = 5;		// [step/s^2]

	bool moving = false;

public:
	bool InitialExtension();

	bool FinalExtension();

	bool CalcAngularVelocity();

	float getAngularvelocityMockup();

	float getArmExtention();
};


extern ArmController armController;

#endif
