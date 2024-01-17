#include "PIDController.hpp"


void PID::config(const PIDParams &params, float limit, bool use_Antiwindup, bool use_DerivativofMeasurment)
{
    this->params = params;
    this->lim = limit;
    this->use_Antiwindup = use_Antiwindup;
    this->use_DerivativofMeasurment = use_DerivativofMeasurment;
}

float PID::calculate(float measurement, float timestamp)
{   
    if (new_params)
    {
        params = getParams();
        lim = getLimit();
        new_params = false;
    }

    // Error
    float error = setpoint - measurement;

    // Proportional term
    float propTerm = params.kp * error;

    if(flagInitialized)
    {
 
        // Delta t in seconds
        float dt = timestamp - this->lastTimestamp;
     
        // Integral term
        float integTerm = 0;
        if (params.ki != 0.f)
        {
            if (!use_Antiwindup || (use_Antiwindup && !saturated)) this->integError += error * dt;
            integTerm = params.ki * this->integError;
        }

        // Derivation term
        float derivTerm = 0;
        if (params.kd != 0.f)
        {
            if (use_DerivativofMeasurment)
            {
                derivTerm = params.kd * (measurement - this->lastMeasurment) / dt;
            }
            else
            {
                derivTerm = params.kd * (error - this->lastError) / dt;
            }
        }


        // Determine output signal
        float controlSignal = propTerm + integTerm + derivTerm;

        // Update state
        this->lastError = error;
        this->lastMeasurment = measurement;
        this->lastTimestamp = timestamp;

        // WARNING
        // temporary values, should be changed
        if (!settled && abs((error - this->lastError) / dt) < 0.1f && abs(error) < 0.1f) settled = true;

        // Limit control signal
        float controlSignalSaturated = controlSignal;
        if (controlSignalSaturated > lim)
        {
            controlSignalSaturated = lim;
            saturated = true;
        }
        else if (controlSignalSaturated < -lim)
        {
            controlSignalSaturated = -lim;
            saturated = true;
        }
        else saturated = false;
        
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

void PID::setParams(const PIDParams& params)
{
    PROTECT_WITH_SEMAPHORE(sem)
    {
        this->params_buffer = params;
        new_params = true;
    }
}


PIDParams PID::getParams()
{
    PIDParams params;
    PROTECT_WITH_SEMAPHORE(sem) params = this->params_buffer;
    return params;
}


void PID::setSetpoint(float setpoint)
{

    this->setpoint = setpoint;
    this->settled = false;
}


float PID::getSetpoint()
{
    return this->setpoint;
}

void PID::setLimit(float limit)
{
    PROTECT_WITH_SEMAPHORE(sem)
    {
        this->lim_buffer = limit;
        new_params = true;
    }
}

float PID::getLimit()
{
    float limit;
    PROTECT_WITH_SEMAPHORE(sem) limit = this->lim_buffer;
    return limit;
}

bool PID::isSettled()
{
    return settled;
}
