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
    this->lastMeasurment = 0.0;
    this->integError = 0.0;
    this->limit = limit;
}



float PID::calculate(float measurement, int64_t timestamp)
{   
    // Save locally to avoid changes during calculations
    PIDParams params = this->getParams();
    float lim = this->getLimits();
    float setp = this->getSetpoint();

    // Error
    float error = setp - measurement;

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
        if(controlSignal > lim) controlSignal = lim;
        else if (controlSignal < -lim) controlSignal = -lim;
        
        return controlSignal;
    } 
    else 
    {
        // Update state
        this->lastError = error;
        this->lastTimestamp = timestamp;
        this->flagInitialized = true;
        
        return propTerm;
    }
    
}



void PID::setParams(const PIDParams &params)
{
    PROTECT_WITH_SEMAPHORE(sem) this->parameters = params;
}



PIDParams PID::getParams()
{
    PIDParams params;
    PROTECT_WITH_SEMAPHORE(sem) params = this->parameters;
    return params;
}


void PID::setSetpoint(float setpoint)
{
    PROTECT_WITH_SEMAPHORE(sem) this->setpoint = setpoint;
}


float PID::getSetpoint()
{
    float setp;
    PROTECT_WITH_SEMAPHORE(sem) setp = this->setpoint;
    return setp;
}


void PID::setLimits(float limit)
{
    PROTECT_WITH_SEMAPHORE(sem) this->limit = limit;
}



float PID::getLimits()
{
    float lim;
    PROTECT_WITH_SEMAPHORE(sem) lim = this->limit;
    return lim;
}
