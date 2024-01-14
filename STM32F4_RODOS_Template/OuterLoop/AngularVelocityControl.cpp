#include "AngularVelocityControl.hpp"

extern Topic<float> AngularVelocitySetpointTopic(-1, "AngularVelocity Setpoint Topic");


float AngularVelocityControl::update(TimestampedData<Attitude_Data> attitude_measured)
{
    return PID::calculate(attitude_measured.data.angularVelocity.z, attitude_measured.timestamp);
}


AngularVelocityControl velocitycontrol;