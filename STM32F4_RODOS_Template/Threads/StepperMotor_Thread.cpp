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
            PROTECT_WITH_SEMAPHORE(sem)
            {
                // Check if Arm is within limits limits (0 < stepCounter < MAX_STEPS) and if so, set direction pin accordingly;
                // If not, then break inner while loop and indicate status as ready
                if (currentDirection)                    // Positive direction
                {
                    if (stepCounter < max_steps)         // Check limit
                    {
                        DirectionPin.setPins(1);
                    }
                    else {
                        stepsToDo = 0;
                        break;
                    }
                }
                else {                                // Negative direction
                    if (stepCounter > 0)                 // Check limit
                    {
                        DirectionPin.setPins(0);
                    }
                    else {
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
                if (currentDirection)
                {
                    this->stepCounter = +1;
                }
                else {
                    this->stepCounter = -1;
                }

                // Update steps to be performed
                this->stepsToDo = -1;
            }

            // Wait for current this->period of time
            suspendCallerUntil(NOW() + period * MICROSECONDS);
        }

        // Update status after execution of all commaned steps
        PROTECT_WITH_SEMAPHORE(sem) this->status_ready = true;

        // Wait for new commands setting this->stepsToDo
        suspendCallerUntil(NOW() + 1 * SECONDS);
    }
}


uint16_t StepperMotorThread::getStepCounter()
{
    uint16_t cnt;
    PROTECT_WITH_SEMAPHORE(sem) cnt = this->stepCounter;
    return cnt;
}



void StepperMotorThread::setDirection(bool direction)
{
    PROTECT_WITH_SEMAPHORE(sem) this->currentDirection = direction;
}



void StepperMotorThread::setPeriod(uint16_t period)
{
    PROTECT_WITH_SEMAPHORE(sem) this->period = period;
}


uint16_t StepperMotorThread::getPeriod()
{
    uint16_t t;
    PROTECT_WITH_SEMAPHORE(sem) t = this->period;
    return t;
}


void StepperMotorThread::setStepsToDo(uint16_t steps)
{
    PROTECT_WITH_SEMAPHORE(sem)
    {
        this->stepsToDo = steps;
        this->status_ready = false;
    }
}

uint16_t StepperMotorThread::getStepsToDo()
{
    uint16_t steps;
    PROTECT_WITH_SEMAPHORE(sem) steps = this->stepsToDo;
    return steps;
}


bool StepperMotorThread::getStatus()
{
    bool status;
    PROTECT_WITH_SEMAPHORE(sem) status = this->status_ready;
    return status;
}



/// @todo UPDATE PINS !!!
StepperMotorThread steppermotorthread(RODOS::GPIO_000, RODOS::GPIO_001);
