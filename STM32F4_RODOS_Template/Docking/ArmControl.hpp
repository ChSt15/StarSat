#ifndef FLOATSAT_DOCKING_ARMCONTROL_HPP_
#define FLOATSAT_DOCKING_ARMCONTROL_HPP_

#include "rodos.h"
#include "matlib.h"
#include <math.h>
#include "StepperMotorThread.hpp"
#include "StepperMotorTopics.hpp"
#include "DockingTopics.hpp"
#include "../Communication/Camera.hpp"


class ArmController
{
private:

	int max_vel;			// [step/s]
	int min_vel;			// [step/s]
	int max_accel;			// [step/s^2]
	int deccel_margin;		// [steps]
	float steps2mm;			// [mm/steps]

	float last_yaw = NAN;
	float last_time;

	StepperInstruction instructions;
	StepperStatus status;
	DockingTememetry telemetry;

	bool moving = false;
	bool deccel = false;

public:

	void config(int max_vel, int min_vel, int max_accel, int deccel_margin, float steps2mm);

	bool InitialExtension(TelemetryCamera& camera);

	bool FinalExtension(TelemetryCamera& camera);

	void CalcAngularVelocity(TelemetryCamera& camera);

	void updateTelemetry(TelemetryCamera& camera);

	// Hard stop, use not recommended. Its there to make sure theres no dual input
	void Stop();

	bool Calibrate();
};


extern ArmController armController;

#endif
