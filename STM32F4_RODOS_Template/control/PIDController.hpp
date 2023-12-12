#ifndef FLOATSAT_CONTROL_PIDCONTROLLER_HPP_
#define FLOATSAT_CONTROL_PIDCONTROLLER_HPP_

#include "rodos.h"
#include "matlib.h"
#include "../threadsafe.hpp"

/**
 * @brief Struct for controller parameters 
*/
struct PIDParams {
    float kp;
    float ki;
    float kd;   
};


class PID {

    private:

        /// @brief Controller parameters
        Threadsafe<PIDParams> parameters;

        /// @brief Desired value the controller should achieve
        Threadsafe<float> setpoint;

        /// @brief Error of last run for determining the derivation of the error
        float lastError;

        /// @brief Integration of error until last control signal determination
        float integError;

        /// @brief Timestamp of last control signal determination in nanoseconds            
        int64_t lastTimestamp;

        /// @brief Limits for control signal
        Threadsafe<float> limit;

        /// @brief Indicates if derivation and integration term can be included -> after lastTimestamp is initialized
        bool flagInitialized = false;

        
    public:

        /**
         * @brief Constructor
        */
       PID();


       /**
        * @brief Initialize parameters
        */
       void init(const PIDParams &params, float limit);


        /**
         * @brief Calculate new control output based on current setpoint and current measurement
         * @return Control output within minLimit to maxLimit
        */
        float calculate(float measurement, int64_t dt);


        /**
         * @brief Set parameter of controller
        */
        void setParams(const PIDParams &params);


        /**
         * @brief Get parameter of controller
        */
        PIDParams getParams();


        /**
         * @brief Set setpoint of controller
        */
        void setSetpoint(float setpoint);


        /**
         * @brief Set max. and min. limit of control signal
        */
        void setLimits(float Limit);


        /**
         * @brief Get max. and min. limit of control signal
        */
        float getLimits();
};


#endif