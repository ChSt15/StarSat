#include "AngularPositionControl.hpp"
#include "rodos.h"



AngularPositionControl::AngularPositionControl()
{
    this->controller = PID();
}



void AngularPositionControl::init(const PIDParams& params, float maxAngularVelocity, bool use_BackCalculation, bool use_DerivativofMeasurment)
{
    this->controller.init(params, maxAngularVelocity, use_BackCalculation, use_DerivativofMeasurment);
}



void AngularPositionControl::setParams(PIDParams params)
{
    this->controller.setParams(params);
}



PIDParams AngularPositionControl::getParams()
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
    return this->controller.getLimits();
}



void AngularPositionControl::setMaxAngularVelocity(float maxAngularVelocity)
{
    this->controller.setLimits(maxAngularVelocity);
}



float AngularPositionControl::update(TimestampedData<Attitude_Data> attitude_measured)
{
    float controlSignal = this->controller.calculate(attitude_measured.data.attitude.toYPR().yaw, attitude_measured.timestamp);
    /**
     * If necessary, add/adjust things like integral windup, etc.
    */
}


AngularPositionControl positionControl;