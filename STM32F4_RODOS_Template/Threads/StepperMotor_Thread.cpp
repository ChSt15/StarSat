#include "StepperMotor_Thread.hpp"


void StepperMotorThread::init()
{
    DirectionPin.init(true, 1, 1);
    StepPin.init(true, 1, 0);
}


void StepperMotorThread::run()
{

    while(true)
    {

  
        while (stepsToDo > 0 and period != 0)
        {
            // temporary
            sem.enter();


            // Check if Arm is within limits limits (0 < stepCounter < MAX_STEPS) and if so, set direction pin accordingly;
            // If not, then break inner while loop and indicate status as ready
            if(currentDirection)                    // Positive direction
            {
                if(stepCounter < max_steps)         // Check limit
                {
                    DirectionPin.setPins(1);
                } else {
                    stepsToDo = 0;
                    break;
                }
            } else {                                // Negative direction
                if(stepCounter > 0)                 // Check limit
                {
                    DirectionPin.setPins(0);
                } else {
                    stepsToDo = 0;
                    break;
                }
            }

            // See Datasheet p.57 11.1 Timing -> Consider minimum DIR to STEP setup time and minimum STEP low time
            StepPin.setPins(0);
            suspendCallerUntil(NOW() + 100 * MICROSECONDS);
            
            // Create Rising Edge
            StepPin.setPins(1);

            // Update current position
            if(currentDirection)
            {
                this->stepCounter =+ 1;
            } else {
                this->stepCounter =- 1;
            }

            // Update steps to be performed
            this->stepsToDo =- 1;

            // temporary
            sem.leave();

            // Wait for current this->period of time
            suspendCallerUntil(NOW() + period * MICROSECONDS);
        }

        // temporary
        sem.enter();

        // Update status after execution of all commaned steps
        this->status_ready = true;

        // temporary
        sem.leave();

        // Wait for new commands setting this->stepsToDo
        suspendCallerUntil(NOW() + 1 * SECONDS);
    }
}


uint16_t StepperMotorThread::getStepCounter()
{
    uint16_t cnt;
    sem.enter();
    cnt = this->stepCounter;
    sem.leave();
    return cnt;
}



void StepperMotorThread::setDirection(bool direction)
{
    sem.enter();
    this->currentDirection = direction;
    sem.leave();
}



void StepperMotorThread::setPeriond(uint16_t period)
{
    sem.enter();
    this->period = period;
    sem.leave();
}



void StepperMotorThread::setStepsToDo(uint16_t steps)
{
    sem.enter();
    this->stepsToDo = steps;
    this->status_ready = false;
    sem.leave();
}



bool StepperMotorThread::getStatus()
{
    bool status;
    sem.enter();
    status = this->status_ready;
    sem.leave();
    return status;
}



/// @todo UPDATE PINS !!!
StepperMotorThread steppermotorthread(RODOS::GPIO_000, RODOS::GPIO_001);
