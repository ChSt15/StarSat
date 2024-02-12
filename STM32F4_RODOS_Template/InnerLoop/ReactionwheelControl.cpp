#include "ReactionwheelControl.hpp"

void ReactionwheelControl::config(const PIDParams &params, float limit, bool use_Antiwindup, bool use_DerivativofMeasurment, float base_vel)
{
    this-> base_vel = base_vel;
    PID::config(params, limit, use_Antiwindup, use_DerivativofMeasurment);
}

void ReactionwheelControl::setSetpoint(float setpoint)
{
    PID::setSetpoint(setpoint + base_vel);
}

float ReactionwheelControl::update(TimestampedData<float> speed_measured)
{
    return PID::calculate(speed_measured.data, speed_measured.timestamp) / 12.f;
}


void ReactionwheelControl::setSaturation(bool sat)
{ 
    saturated = sat;
}

ReactionwheelControl reactionwheelControl;