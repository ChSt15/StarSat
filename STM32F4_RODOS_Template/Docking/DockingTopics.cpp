#include "DockingTopics.hpp"

Topic<StepperInstruction> stepperPeriodTopic = Topic<StepperInstruction>(-1, "Stepper Instructions");

Topic<StepperStatus> stepperCntTopic = Topic<StepperStatus>(-1, "Stepper Status");

Topic<DockingTememetry> dockingTelemetryTopic = Topic<DockingTememetry>(-1, "Docking Telemetry");