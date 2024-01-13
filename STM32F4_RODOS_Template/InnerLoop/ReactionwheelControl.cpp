#include "ReactionwheelControl.hpp"


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
    return PID::calculate(speed_measured.data, speed_measured.timestamp) / 12.f;
}

ReactionwheelControl reactionwheelControl;