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
    this->limit = limit;
}



float PID::calculate(float measurement, int64_t timestamp)
{   
    // Save locally to avoid changes during calculations
    sem.enter();
    PIDParams params = this->parameters;
    float lim = this->limit;
    float setp = this->setpoint;
    sem.leave();

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
    sem.enter();
    this->parameters = params;
    sem.leave();
}



PIDParams PID::getParams()
{
    PIDParams params;
    sem.enter();
    params = this->parameters;
    sem.leave();
    return params;
}



void PID::setSetpoint(float setpoint)
{
    sem.enter();
    this->setpoint = setpoint;
    sem.leave();
}



void PID::setLimits(float limit)
{
    sem.enter();
    this->limit = limit;
    sem.leave();
}



float PID::getLimits()
{
    float lim;
    sem.enter();
    lim = this->limit;
    sem.leave();
    return lim;
}
