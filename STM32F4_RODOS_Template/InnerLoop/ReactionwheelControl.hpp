#ifndef FLOATSAT_INNERLOOP_REACTIONWHEELCONTROL_HPP_
#define FLOATSAT_INNERLOOP_REACTIONWHEELCONTROL_HPP_

#include "rodos.h"
#include "../PIDController.hpp"
#include "../timestamp.hpp"


class ReactionwheelControl: public PID
{
public:

	void config(const PIDParams &params, float limit, bool use_Antiwindup, bool use_DerivativofMeasurment, float base_vel);

	/**
	 * @brief Determine output of reactionwheel controller / input of HBridge
	 * @param speed_measured: measurement of current speed of reaction wheel measured by Encoder in [rad/s]
	 * @return Percentage of max. voltage that needs to be applied by HBridge; range of -1 to 1
	*/
	float update(TimestampedData<float> speed_measured);

	void setSetpoint(float setpoint);

	void setSaturation(bool sat);

private:

	float base_vel = 0;

};


extern ReactionwheelControl reactionwheelControl;

#endif
