#ifndef FLOATSAT_THREADS_STEPPERMOTORTHREAD_HPP_
#define FLOATSAT_THREADS_STEPPERMOTORTHREAD_HPP_

#include "rodos.h"
#include "../timestamp.hpp"
#include "../threadsafe.hpp"


class StepperMotorThread : public Thread
{

public:

	// Set name, prio and stack size
	StepperMotorThread(RODOS::GPIO_PIN dir_pin, RODOS::GPIO_PIN step_pin) : 
                Thread("Sensor Thread", 100, 2000), 
                DirectionPin(dir_pin), 
                StepPin(step_pin) 
                {
                    stepCounter = 0;
                    currentDirection = true;
                    period = 0;
                    status_ready = true;
                    stepsToDo = 0;
                }


	void init();
	void run();


    /**
     * @brief Returns current position of arm in steps
    */
    uint16_t getStepCounter();


    /**
     * @brief Gets status indicating if all commanded steps are executed
    */
    bool getStatus();


    /**
     * @brief Set step direction; True: positive direction, False: negative direction
    */
   void setDirection(bool direction);


    /**
     * @brief Set period to wait between two steps [us]
    */
   void setPeriond(uint16_t period);


   /**
     * @brief Sets number of steps, which needs to be performed
    */
    void setStepsToDo(uint16_t steps);


private:

    HAL_GPIO DirectionPin;                                  // High: positive direction, Low: negative direction
    HAL_GPIO StepPin;                                       // Rising Edge indicates steps

    Threadsafe<uint16_t> stepCounter;                       // Indicates current position of arm
    Threadsafe<bool> currentDirection;                      // True: positive direction, False: negative direction
    Threadsafe<uint16_t> period;                            // Time period between two steps in microseconds
    Threadsafe<bool> status_ready;                          // Indicates if all commanded steps are executed
    Threadsafe<uint16_t> stepsToDo;                         // Indicates how many steps still needs to be executed

    /// @todo ADJUST VALUE
    uint16_t max_steps = 10000;                             // Max. number of steps the motor can execute


};


extern StepperMotorThread sensorthread;

#endif 