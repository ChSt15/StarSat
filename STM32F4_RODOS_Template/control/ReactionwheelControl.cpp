#include "ReactionwheelControl.hpp"
#include "rodos.h"



ReactionwheelControl::ReactionwheelControl()
{
    this->controller = PID();
}



void ReactionwheelControl::init(const PIDParams& params, float maxLimit, float minLimit, float maxVoltage)
{
    this->controller.init(params, maxLimit, minLimit);
    this->maxVoltage = maxVoltage;
}



void ReactionwheelControl::setParams(const PIDParams& params)
{
    this->controller.setParams(params);
}



const PIDParams& ReactionwheelControl::getParams()
{
    return this->controller.getParams();
}


void ReactionwheelControl::setDesiredSpeed(float w_set)
{
    this->controller.setSetpoint(w_set);
}



float ReactionwheelControl::update(TimestampedData<float> speed)
{
    float controlSignal = this->controller.calculate(speed.data, speed.timestamp);
    controlSignal = controlSignal / maxVoltage;
}

ReactionwheelControl reactionwheelControl;