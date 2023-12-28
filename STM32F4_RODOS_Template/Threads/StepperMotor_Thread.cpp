#include "StepperMotor_Thread.hpp"


void StepperMotorThread::init()
{
    DirectionPin.init(true, 1, 1);
    StepPin.init(true, 1, 0);
}


void StepperMotorThread::run()
{
    bool status;
    
    while(true)
    {
        PROTECT_WITH_SEMAPHORE(sem) status = this->status_calib;
        if (status)
        {

            while (stepsToDo > 0 && period != 0)
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
                }

                // See Datasheet p.57 11.1 Timing -> Consider minimum DIR to STEP setup time and minimum STEP low time
                StepPin.setPins(0);
                suspendCallerUntil(NOW() + 100 * MICROSECONDS);

                // Create Rising Edge
                StepPin.setPins(1);

                // Update current position
                if (currentDirection)
                {
                    PROTECT_WITH_SEMAPHORE(sem) this->stepCounter++;
                }
                else {
                    PROTECT_WITH_SEMAPHORE(sem) this->stepCounter--;
                }

                // Update steps to be performed
                PROTECT_WITH_SEMAPHORE(sem) this->stepsToDo--;


                // Wait for current this->period of time
                suspendCallerUntil(NOW() + period * MICROSECONDS - 100 * MICROSECONDS);
            }

            // Update status after execution of all commaned steps
            PROTECT_WITH_SEMAPHORE(sem) this->status_ready = true;

            // Wait for new commands setting this->stepsToDo
            suspendCallerUntil(END_OF_TIME);
        }
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



void StepperMotorThread::setPeriod(int period)
{
    PROTECT_WITH_SEMAPHORE(sem) this->period = period;
}


int StepperMotorThread::getPeriod()
{
    int t;
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


bool StepperMotorThread::calibrate()
{
    PROTECT_WITH_SEMAPHORE(sem)
    {
        while(!status_calib)
        {
            if(CalibPin.readPins() == 0)                // Check if Pin is low -> then execute step backwards
            {
                DirectionPin.setPins(0);
                StepPin.setPins(0);
                suspendCallerUntil(NOW() + 100 * MICROSECONDS);
                StepPin.setPins(1);
                suspendCallerUntil(NOW() + 500 * MICROSECONDS);
            } else {                                    // Pin is high -> calibration completed
                status_calib = true;
            }   
        }

        StepPin.setPins(0);
        DirectionPin.setPins(1);
        stepCounter = 0;
    }

    bool status;
    PROTECT_WITH_SEMAPHORE(sem) status = this->status_calib;
    return status;
}



/// @todo UPDATE PINS !!!
StepperMotorThread steppermotorthread(RODOS::GPIO_049, RODOS::GPIO_051, RODOS::GPIO_001);
