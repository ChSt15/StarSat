#include "AngularVelocityControl.hpp"
#include "rodos.h"



AngularVelocityControl::AngularVelocityControl()
{
    this->controller = PID();
}



void AngularVelocityControl::init(const PIDParams& params, float maxSpeed, float maxDesiredVelocity, bool use_BackCalculation, bool use_DerivativofMeasurment)
{
    this->controller.init(params, maxSpeed, use_BackCalculation, use_DerivativofMeasurment);
    this->maxDesiredVelocity = maxDesiredVelocity;
}



void AngularVelocityControl::setParams(PIDParams params)
{
    this->controller.setParams(params);
}



PIDParams AngularVelocityControl::getParams()
{
    return this->controller.getParams();
}



void AngularVelocityControl::setDesiredAngularVelocity(float w_set)
{
    if(abs(w_set) >= this->maxDesiredVelocity) 
    {
        this->controller.setSetpoint(maxDesiredVelocity);
    } else {
        this->controller.setSetpoint(w_set);
    }
}



float AngularVelocityControl::getMaxSpeed()
{
    return this->controller.getLimits();
}



void AngularVelocityControl::setMaxSpeed(float maxSpeed)
{
    this->controller.setLimits(maxSpeed);
}


float AngularVelocityControl::getMaxDesiredVelocity()
{
    return this->maxDesiredVelocity;
}



float AngularVelocityControl::update(TimestampedData<Attitude_Data> attitude_measured)
{
    return this->controller.calculate(attitude_measured.data.angularVelocity.z, attitude_measured.timestamp);
}


AngularVelocityControl velocitycontrol;