#include "ReactionwheelControl.hpp"
#include "rodos.h"



ReactionwheelControl::ReactionwheelControl()
{
    this->controller = PID();
}



void ReactionwheelControl::init(const PIDParams& params, float maxVoltage, float maxDesiredSpeed, bool use_BackCalculation, bool use_DerivativofMeasurment)
{
    this->controller.init(params, maxVoltage, use_BackCalculation, use_DerivativofMeasurment);
    this->maxDesiredSpeed = maxDesiredSpeed;
}



void ReactionwheelControl::setParams(PIDParams params)
{
    this->controller.setParams(params);
}



PIDParams ReactionwheelControl::getParams()
{
    return this->controller.getParams();
}



void ReactionwheelControl::setDesiredSpeed(float w_set)
{   
    if(abs(w_set) >= this->maxDesiredSpeed) 
    {
        this->controller.setSetpoint(maxDesiredSpeed);
    } else {
        this->controller.setSetpoint(w_set);
    }
}



float ReactionwheelControl::getMaxVoltage()
{
    return this->controller.getLimits();
}



void ReactionwheelControl::setMaxVoltage(float maxVoltage)
{
    this->controller.setLimits(maxVoltage);
}


float ReactionwheelControl::getMaxDesiredSpeed()
{
    return this->maxDesiredSpeed;
}



float ReactionwheelControl::update(TimestampedData<float> speed_measured)
{
    float controlSignal = this->controller.calculate(speed_measured.data, speed_measured.timestamp);
    float maxVoltage = this->controller.getLimits();
    return controlSignal / maxVoltage;
}

ReactionwheelControl reactionwheelControl;