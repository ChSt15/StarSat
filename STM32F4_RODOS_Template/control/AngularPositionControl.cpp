#include "AngularPositionControl.hpp"


AngularPositionControl::AngularPositionControl()
{

}

void AngularPositionControl::setParams(const PositionControlParams& params)
{
    controlParams = params;
}

const PositionControlParams& AngularPositionControl::getParams()
{
    return controlParams;
}

void AngularPositionControl::setDesiredAngle(float angle_set)
{

}

float AngularPositionControl::getSpeed(float angle_mes)
{

}


AngularPositionControl positionControl;