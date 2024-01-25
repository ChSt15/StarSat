#include "StepperMotorThread.hpp"


// Subscriber for instructions
static CommBuffer<StepperInstruction> instructionBuffer;
static Subscriber instructionSubsciber(stepperInstructionsTopic, instructionBuffer, "Stepper Thread");


void StepperMotorThread::init()
{
    DirectionPin.init(true, 1, 1);
    StepPin.init(true, 1, 0);
    CalibPin.init(false, 0, 0);
    EnablePin.init(true, 1, 0);
}


void StepperMotorThread::run()
{
    while(true)
    {
        if (status.status_calib)
        {
            // Get new instruction if available
            instructionBuffer.getOnlyIfNewData(this->instructions);

            while (instructions.stepTarget != status.stepCounter && instructions.period > 0)
            {
                // Get new instruction if available
                instructionBuffer.getOnlyIfNewData(this->instructions);
                
                // Check if Arm is within limits limits (0 < stepCounter < MAX_STEPS) and if so, set direction pin accordingly;
                // If not, then break inner while loop and indicate status as ready
                if (0 < instructions.stepTarget && instructions.stepTarget < max_steps)
                {
                    if (instructions.stepTarget > status.stepCounter)       // Positive direction
                    {
                        DirectionPin.setPins(1);

                        // See Datasheet p.57 11.1 Timing -> Consider minimum DIR to STEP setup time and minimum STEP low time
                        StepPin.setPins(0);
                        suspendCallerUntil(NOW() + 100 * MICROSECONDS);

                        // Create Rising Edge
                        StepPin.setPins(1);

                        // Update current position
                        status.stepCounter++;
                    }
                    else                                                    // Negative direction
                    {
                        DirectionPin.setPins(0);

                        // See Datasheet p.57 11.1 Timing -> Consider minimum DIR to STEP setup time and minimum STEP low time
                        StepPin.setPins(0);
                        suspendCallerUntil(NOW() + 100 * MICROSECONDS);

                        // Create Rising Edge
                        StepPin.setPins(1);

                        // Update current position
                        status.stepCounter--;
                    }

                    // Publish updated status
                    stepperStatusTopic.publish(status);

                    // Wait for current this->period of time
                    suspendCallerUntil(NOW() + instructions.period * MICROSECONDS - 100 * MICROSECONDS);
                }
                else break;
            }

            // Update status after execution of all commaned steps
            status.status_execution = true;

            // Wait until woken up
            suspendCallerUntil(END_OF_TIME);
        }

        // Wait until woken up
        suspendCallerUntil(END_OF_TIME);
    }
}

bool StepperMotorThread::calibrate()
{
    status.status_calib = false;

    while(!status.status_calib)
    {
        if(CalibPin.readPins() == 0)                // Check if Pin is low -> then execute step backwards
        {
            DirectionPin.setPins(0);
            StepPin.setPins(0);
            suspendCallerUntil(NOW() + 100 * MICROSECONDS);
            StepPin.setPins(1);
            suspendCallerUntil(NOW() + 500 * MICROSECONDS);
        } 
        else status.status_calib = true;            // Pin is high -> calibration completed
    }

    StepPin.setPins(0);
    DirectionPin.setPins(1);
    status.stepCounter = 0;

    return status.status_calib;
}


StepperMotorThread steppermotorthread(RODOS::GPIO_049, RODOS::GPIO_051, RODOS::GPIO_001, RODOS::GPIO_055);