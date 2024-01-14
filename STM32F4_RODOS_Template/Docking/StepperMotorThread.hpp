#ifndef FLOATSAT_DOCKING_STEPPERMOTORTHREAD_HPP_
#define FLOATSAT_DOCKING_STEPPERMOTORTHREAD_HPP_

#include "rodos.h"

#include "StepperMotorTopics.hpp"


class StepperMotorThread : public Thread
{
public:


	// Set name, prio and stack size
	StepperMotorThread(RODOS::GPIO_PIN dir_pin, RODOS::GPIO_PIN step_pin, RODOS::GPIO_PIN calib_pin, RODOS::GPIO_PIN enable_pin)  :
                Thread("StepperMotor Thread", 200, 1000),
                DirectionPin(dir_pin),
                StepPin(step_pin),
                CalibPin(calib_pin),
                EnablePin(enable_pin)
                {
                    status.stepCounter = 0;
                    instructions.period = 0;
                    status.status_execution = true;
                    status.status_calib = false;
                }


	void init();

	void run();

    /**
     * @brief Sets number of steps, which needs to be performed
    */
    bool calibrate();


private:

    HAL_GPIO DirectionPin;                                  // High: positive direction, Low: negative direction
    HAL_GPIO StepPin;                                       // Rising Edge indicates steps
    HAL_GPIO CalibPin;                                      // High if Arm is at limit (= calibrated)
    HAL_GPIO EnablePin;                                     // High: Motor is enabled, Low: Motor is disabled

    StepperStatus status;
    StepperInstruction instructions;
 
    /// @todo ADJUST VALUE
    const uint16_t max_steps = 10000;                       // Max. number of steps the motor can execute

};


extern StepperMotorThread steppermotorthread;

#endif 
