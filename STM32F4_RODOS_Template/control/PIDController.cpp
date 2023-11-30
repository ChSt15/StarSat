#include "PIDController.hpp"
#include "rodos.h"


PID::PID()
{

}


void PID::init(const PIDParams &params, float maxLimit, float minLimit)
{
    this->parameters = params;
    this->setpoint = 0.0;
    this->lastError = 0.0;
    this->integError = 0.0;
    this->maxLimit = maxLimit;
    this->minLimit = minLimit;
}



float PID::calculate(float measurement, int64_t timestamp)
{   
    // Error
    float error = this->setpoint - measurement;

    // Proportional term
    float propTerm = parameters.kp * error;

    if(flagInitialized)
    {
        // Delta t in nanoseconds   ->  might need to be adjusted
        int64_t dt = timestamp - this->lastTimestamp;

        // Integral term
        this->integError += error * dt;
        float integTerm = parameters.ki * this->integError;

        // Derivation term
        float derivTerm = parameters.kd * (error - this->lastError) / dt;

        // Determine output signal
        float controlSignal = propTerm + integTerm + derivTerm;

        // Update state
        this->lastError = error;
        this->lastTimestamp = timestamp;

        // Limit control signal
        if(controlSignal > this->maxLimit) controlSignal = this->maxLimit;
        else if (controlSignal < this->minLimit) controlSignal = this->minLimit;
        
        return controlSignal;
    } else {
        // Update state
        this->lastError = error;
        this->lastTimestamp = timestamp;
        this->flagInitialized = true;
        
        return propTerm;
    }
    
}



void PID::setParams(const PIDParams &params)
{
    this->parameters = params;
}


PIDParams& PID::getParams()
{
    return this->parameters;
}



void PID::setSetpoint(float setpoint)
{
    this->setpoint = setpoint;
}



void PID::setLimits(float maxLimit, float minLimit)
{
    this->maxLimit = maxLimit;
    this->minLimit = minLimit;
}