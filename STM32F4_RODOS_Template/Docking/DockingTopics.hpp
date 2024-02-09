#ifndef FLOATSAT_DOCKING_DOCKINGTOPICS_HPP_
#define FLOATSAT_DOCKING_DOCKINGTOPICS_HPP_

#include "rodos.h"


struct DockingTememetry
{
	float armVelocity = 0;
	float armExtention = 0;
	float mockupAngularvelocity = 0;
	float mockupDistance = 0;
	float mockupYaw = 0;
	float Yaw2Mockup = 0;
};

extern Topic<DockingTememetry> dockingTelemetryTopic;


#endif