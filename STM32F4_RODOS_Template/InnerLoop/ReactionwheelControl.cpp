#include "ReactionwheelControl.hpp"


float ReactionwheelControl::update(TimestampedData<float> speed_measured)
{
    return PID::calculate(speed_measured.data, speed_measured.timestamp) / 12.f;
}


ReactionwheelControl reactionwheelControl;