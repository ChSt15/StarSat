#include "AngularPositionControl.hpp"
#include "rodos.h"



AngularPositionControl::AngularPositionControl()
{
    this->controller = PID();
}



void AngularPositionControl::init(const PIDParams& params, float maxAngularVelocity)
{
    this->controller.init(params, maxAngularVelocity, -maxAngularVelocity);
    this->maxAngularVelocity = maxAngularVelocity;
}



void AngularPositionControl::setParams(const PIDParams& params)
{
    this->controller.setParams(params);
}



const PIDParams& AngularPositionControl::getParams()
{
    return this->controller.getParams();
}



void AngularPositionControl::setDesiredAngle(float angle_set)
{   

    if(angle_set >= 2 * M_PI)
    {   
        while(angle_set >= 2 * M_PI)
        {
            angle_set = angle_set - 2 * M_PI;
        }
    } else if (angle_set <= - 2 * M_PI)
    {
        while(angle_set <= - 2 * M_PI)
        {
            angle_set = angle_set + 2 * M_PI;
        }
    }

    this->controller.setSetpoint(angle_set);
}



float AngularPositionControl::getMaxAngularVelocity()
{
    return this->maxAngularVelocity;
}



void AngularPositionControl::setMaxAngularVelocity(float maxAngularVelocity)
{
    this->controller.setLimits(maxAngularVelocity, -maxAngularVelocity);
    this->maxAngularVelocity = maxAngularVelocity;
}



float AngularPositionControl::update(TimestampedData<float> angle_measured)
{
    float controlSignal = this->controller.calculate(angle_measured.data, angle_measured.timestamp);
    /**
     * If necessary, add/adjust things like integral windup, etc.
    */
}


AngularPositionControl positionControl;