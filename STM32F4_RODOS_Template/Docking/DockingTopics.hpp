#pragma once

#include "rodos.h"

struct StepperInstruction
{
	int period = 0;
	int stepTarget = 0;
	bool direction = true;
};

struct StepperStatus
{
	int stepCounter = 0;
	int period = 0;
	int steps2Do = 0;
	bool calib = false;
};


struct DockingTememetry
{
	float armVelocity = 0;
	float armPosition = 0;
	float mockupAngularvelocity = 0;
	float mockupDistance = 0;
};

extern Topic<StepperInstruction> stepperInstructionsTopic;
 
extern Topic<StepperStatus> stepperStatusTopic;

extern Topic<DockingTememetry> dockingTelemetryTopic;