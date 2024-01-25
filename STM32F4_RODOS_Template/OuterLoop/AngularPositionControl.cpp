#include "AngularPositionControl.hpp"

extern Topic<float> AngularPositionSetpointTopic(-1, "AngularPosition Setpoint Topic");


void AngularPositionControl::setSetpoint(float angle_set)
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
    PID::setSetpoint(angle_set);
}


float AngularPositionControl::update(TimestampedData<Attitude_Data> attitude_measured)
{
    float measured_pos = attitude_measured.data.attitude.toYPR().yaw;

    // make sure the angle "warp around" by limiting set angle to measured angle +/- pi;
    float set_pos = PID::getSetpoint();
    while (set_pos > measured_pos + M_PI) set_pos -= 2 * M_PI;
    while (set_pos < measured_pos - M_PI) set_pos += 2 * M_PI;
    PID::setSetpoint(set_pos);

    return PID::calculate(measured_pos, attitude_measured.timestamp);
}


AngularPositionControl positionControl;