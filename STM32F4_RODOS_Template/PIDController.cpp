#include "PIDController.hpp"


void PID::config(const PIDParams &params, float limit, bool use_Antiwindup, bool use_DerivativofMeasurment)
{
    this->params = this->params_buffer = params;
    this->lim = this->lim_buffer = limit;
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

    // Delta t in seconds
    float dt = timestamp - this->lastTimestamp;

    //reset after timeout
    if (dt > 1)  flagInitialized = false;

    if(flagInitialized)
    {

        // Derivation term
        float derivTerm = 0;
        if (params.kd != 0.f)
        {
            if (use_DerivativofMeasurment)
            {
                derivTerm = params.kd * -(measurement - this->lastMeasurment) / dt;
            }
            else
            {
                derivTerm = params.kd * (error - this->lastError) / dt;
            }
        }

        // Integral term
        if (use_Antiwindup && saturated) // Anti-windup by subtracting the differnce between the saturated and unsaturated control signal from the integral term
        {
            if (this->integError > 0 && error < 0 || this->integError < 0 && error > 0)
            {
                this->integError += error * dt;
            }
            
        } else 
            this->integError += error * dt;

        // Determine output signal
        float controlSignal = propTerm + this->integError * params.ki + derivTerm;
        

        // Update state
        this->lastError = error;
        this->lastMeasurment = measurement;
        this->lastTimestamp = timestamp;

        // WARNING
        // temporary values, should be changed
        if (!settled && abs((error - this->lastError) / dt) < 0.5f && abs(error) < 3.f) settled = true;

        if (use_Antiwindup) { // Anti-windup by subtracting the differnce between the saturated and unsaturated control signal from the integral term

            /*if (controlSignal > lim) {
                float diff = controlSignal - lim;
                controlSignal = lim;
                if (diff > lim) 
                    diff = lim;
                this->integError -= diff;
            } else if (controlSignal < -lim) {
                float diff = controlSignal + lim;
                controlSignal = -lim;
                if (diff < -lim) 
                    diff = -lim;
                this->integError -= diff;
            }*/
        }

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
