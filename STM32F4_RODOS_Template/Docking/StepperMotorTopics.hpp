#ifndef FLOATSAT_DOCKING_STEPPERMOTORTOPICS_HPP_
#define FLOATSAT_DOCKING_STEPPERMOTORTOPICS_HPP_


#include "rodos.h"

struct StepperInstruction
{
    int period = 0;                         // Time period between two steps in microseconds
    int stepTarget = 0;                     // Indicates the target position
};

struct StepperStatus
{
    int stepCounter = 0;                    // Indicates current position of arm
    bool status_execution = false;          // Indicates if all commanded steps are executed
    bool status_calib = false;              // Indicates if arm is calibrated
};

extern Topic<StepperInstruction> stepperInstructionsTopic;

extern Topic<StepperStatus> stepperStatusTopic;

#endif