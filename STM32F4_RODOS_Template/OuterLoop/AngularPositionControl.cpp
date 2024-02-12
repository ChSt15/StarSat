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

    auto xAxis = Vector3D(1, 0, 0);
    auto zAxis = Vector3D(0, 0, 1);
    auto angle = PID::getSetpoint();
    auto setpointQ = Quaternion(AngleAxis(angle, zAxis));

    auto errorQ = attitude_measured.data.attitude*(setpointQ.conjugate());

    auto errorYaw = errorQ.toYPR().yaw;

    PID::setSetpoint(0);
    return PID::calculate(errorYaw, attitude_measured.timestamp);

    
    /*float measured_pos = attitude_measured.data.attitude.toYPR().yaw;

    // make sure the angle "warp around" by limiting set angle to measured angle +/- pi;
    float set_pos = PID::getSetpoint();
    while (set_pos > measured_pos + M_PI) set_pos -= 2 * M_PI;
    while (set_pos < measured_pos - M_PI) set_pos += 2 * M_PI;
    PID::setSetpoint(set_pos);

    return PID::calculate(measured_pos, attitude_measured.data.angularVelocity.z, attitude_measured.timestamp);*/
}


AngularPositionControl positionControl;