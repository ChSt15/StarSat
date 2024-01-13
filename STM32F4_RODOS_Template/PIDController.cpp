#include "PIDController.hpp"


void PID::config(const PIDParams &params, float limit_out, float limit_in, bool use_BackCalculation, bool use_DerivativofMeasurment)
{
    this->parameters = params;
    this->limit_out = limit_out;
    this->limit_in = limit_in;
    this->use_BackCalculation = use_BackCalculation;
    this->use_DerivativofMeasurment = use_DerivativofMeasurment;
}

float PID::calculate(float measurement, float timestamp)
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
 
        // Delta t in seconds
        float dt = timestamp - this->lastTimestamp;
     

        // Integral term
        float integTerm = 0;
        if (!use_BackCalculation)
        {
            this->integError += error * dt;
            integTerm = params.ki * this->integError;
        }
        else 
        {
            integTerm = this->integError;
        }

        // Derivation term
        float derivTerm = 0;
        if (use_DerivativofMeasurment)
        {
            derivTerm = params.kd * (measurement - this->lastMeasurment) / dt;
        }
        else
        {
            derivTerm = params.kd * (error - this->lastError) / dt;
        }


        // Determine output signal
        float controlSignal = propTerm + integTerm + derivTerm;

        // Update state
        this->lastError = error;
        this->lastMeasurment = measurement;
        this->lastTimestamp = timestamp;

        // WARNING
        // temporary values, should be changed
        if (!settled && abs((error - this->lastError) / dt) < 0.1f && abs(error) < 0.1f) PROTECT_WITH_SEMAPHORE(sem) settled = true;

        // Limit control signal
        float controlSignalSaturated = controlSignal;
        if(controlSignalSaturated > lim) controlSignalSaturated = lim;
        else if (controlSignalSaturated < -lim) controlSignalSaturated = -lim;
        
        // Backcalculation
        if (use_BackCalculation)
        {
            this->integError += (params.ki * error + controlSignalSaturated - controlSignal) * dt;
        }

        return controlSignalSaturated;
    } 
    else 
    {
        // Update state
        this->lastError = error;
        this->lastMeasurment = measurement;
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
    PROTECT_WITH_SEMAPHORE(sem)
    {
        this->setpoint = setpoint;
        this->settled = false;
    }
}


float PID::getSetpoint()
{
    float setp;
    PROTECT_WITH_SEMAPHORE(sem) setp = this->setpoint;
    return setp;
}


void PID::setOutputLimits(float limit)
{
    PROTECT_WITH_SEMAPHORE(sem) this->limit = limit;
}



float PID::getOutputLimits()
{
    float lim;
    PROTECT_WITH_SEMAPHORE(sem) lim = this->limit;
    return lim;
}


bool PID::isSettled()
{
    bool settled;
    PROTECT_WITH_SEMAPHORE(sem) settled = this->settled;
    return settled;
}
