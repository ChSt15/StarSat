#include "PIDController.hpp"
#include "rodos.h"



PID::PID()
{

}



void PID::init(const PIDParams &params, float limit)
{
    this->parameters = params;
    this->setpoint = 0.0;
    this->lastError = 0.0;
    this->integError = 0.0;
    this->Limit = limit;
}



float PID::calculate(float measurement, int64_t timestamp)
{   
    // Save locally to avoid changes during calculations
    PIDParams params = parameters.get();

    // Error
    float error = this->setpoint.get() - measurement;

    // Proportional term
    float propTerm = params.kp * error;

    if(flagInitialized)
    {
        // Delta t in nanoseconds   ->  might need to be adjusted
        int64_t dt = timestamp - this->lastTimestamp;

        // Integral term
        this->integError += error * dt;
        float integTerm = params.ki * this->integError;

        // Derivation term
        float derivTerm = params.kd * (error - this->lastError) / dt;

        // Determine output signal
        float controlSignal = propTerm + integTerm + derivTerm;

        // Update state
        this->lastError = error;
        this->lastTimestamp = timestamp;

        // Limit control signal
        if(controlSignal > this->Limit) controlSignal = this->Limit;
        else if (controlSignal < -this->Limit) controlSignal = -this->Limit;
        
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
    this->parameters.set(params);
}



PIDParams PID::getParams()
{
    return this->parameters.get();
}



void PID::setSetpoint(float setpoint)
{
    this->setpoint.set(setpoint);
}



void PID::setLimits(float limit)
{
    this->Limit.set(limit);
}

float PID::getLimits()
{
    return this->Limit.get();
}
