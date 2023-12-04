#include "AngularVelocityControl.hpp"
#include "rodos.h"



AngularVelocityControl::AngularVelocityControl()
{
    this->controller = PID();
}



void AngularVelocityControl::init(const PIDParams& params, float maxSpeed, float maxDesiredVelocity)
{
    this->controller.init(params, maxSpeed, -maxSpeed);
    this->maxSpeed = maxSpeed;
    this->maxDesiredVelocity = maxDesiredVelocity;
}



void AngularVelocityControl::setParams(const PIDParams& params)
{
    this->controller.setParams(params);
}



const PIDParams& AngularVelocityControl::getParams()
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
    return this->maxSpeed;
}



void AngularVelocityControl::setMaxSpeed(float maxSpeed)
{
    this->controller.setLimits(maxSpeed, -maxSpeed);
    this->maxSpeed = maxSpeed;
}



void AngularVelocityControl::setMaxDesiredVelocity(float maxDesiredVelocity)
{
    this->maxDesiredVelocity = maxDesiredVelocity;
}



float AngularVelocityControl::getMaxDesiredVelocity()
{
    return this->maxDesiredVelocity;
}



float AngularVelocityControl::update(TimestampedData<float> velocity_measured)
{
    float controlSignal = this->controller.calculate(velocity_measured.data, velocity_measured.timestamp);
    /**
     * If necessary, add/adjust things like integral windup, etc.
    */
}


AngularVelocityControl velocitycontrol;