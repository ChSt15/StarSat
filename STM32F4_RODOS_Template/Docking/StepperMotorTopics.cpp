#include "StepperMotorTopics.hpp"


Topic<StepperInstruction> stepperInstructionsTopic(-1, "Stepper Instructions");

Topic<StepperStatus> stepperStatusTopic(-1, "Stepper Status");