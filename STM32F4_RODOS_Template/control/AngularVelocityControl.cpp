#include "AngularVelocityControl.hpp"


AngularVelocityControl::AngularVelocityControl()
{

}

void AngularVelocityControl::setParams(const VelocityControlParams& params)
{
    controlParams = params;
}

const VelocityControlParams& AngularVelocityControl::getParams()
{
    return controlParams;
}

void AngularVelocityControl::setDesiredAngularVelocity(float w_set)
{
    
}

float AngularVelocityControl::getSpeed(float w_mes)
{

}


AngularVelocityControl velocitycontrol;