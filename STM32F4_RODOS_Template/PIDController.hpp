#ifndef FLOATSAT_PIDCONTROLLER_HPP_
#define FLOATSAT_PIDCONTROLLER_HPP_

#include "rodos.h"
#include "matlib.h"
#include <cmath>

/**
 * @brief Struct for controller parameters 
*/
struct PIDParams 
{
    float kp = 0.f;
    float ki = 0.f;
    float kd = 0.f;   
};


class PID {

    private:

        Semaphore sem;

        /// @brief Controller parameters
        PIDParams params;
        PIDParams params_buffer;

        /// @brief Limits for control signal
        float lim = 0.f;
        float lim_buffer = 0.f;

        bool new_params = false;

        /// @brief Desired value the controller should achieve
        float setpoint = 0.f;

        /// @brief Error of last run for determining the derivation of the error
        float lastError = 0.f;

        /// @brief Measurment of last run for determining the derivation of the Measurment
        float lastMeasurment = 0.f;

        /// @brief Integration of error until last control signal determination
        float integError = 0.f;

        /// @brief Timestamp of last control signal determination in seconds            
        float lastTimestamp = 0.f;


        /// @brief Indicates if derivation and integration term can be included -> after lastTimestamp is initialized
        bool flagInitialized = false;
         
        bool use_Antiwindup = false;

        bool use_DerivativofMeasurment = false;

        bool settled = false;

        bool saturated = false;
  
    public:

       /**
        * @brief Initialize parameters
        */
       void config(const PIDParams &params, float limit, bool use_Antiwindup, bool use_DerivativofMeasurment);


        /**
         * @brief Calculate new control output based on current setpoint and current measurement
         * @return Control output within minLimit to maxLimit
        */
        float calculate(float measurement, float dt);


        /**
         * @brief Set parameter of controller
        */
        void setParams(const PIDParams& params);


        /**
         * @brief Get parameter of controller
        */
        PIDParams getParams();


        /**
         * @brief Set setpoint of controller
        */
        void setSetpoint(float setpoint);

        float getLimit();

        void setLimit(float limit);


        float getSetpoint();

        bool isSettled();
};


#endif
