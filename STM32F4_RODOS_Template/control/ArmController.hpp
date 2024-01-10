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

	float w_Mockup = NAN;
	float distance;

	int max_vel;			// [step/s]
	int min_vel;			// [step/s]
	int max_accel;			// [step/s^2]
	int deccel_margin;		// [steps]

	bool moving = false;
	bool deccel = false;

	int lastframe = -42;

public:

	void config(int max_vel, int min_vel, int max_accel, int deccel_margin);

	bool InitialExtension(TelemetryCamera& camera);

	bool FinalExtension(TelemetryCamera& camera);

	void CalcAngularVelocity(TelemetryCamera& camera);

	float getAngularvelocityMockup();

	float getArmExtention();
};


extern ArmController armController;

#endif
