#ifndef FLOATSAT_COMMUNICATION_TELEMETRY_HPP_
#define FLOATSAT_COMMUNICATION_TELEMETRY_HPP_

#include "rodos.h"

#include "../hardware/imu.hpp"
#include "Gateway.hpp"

class Telemety
{
private:

	TopicListReport topics;

public:

	Telemety();

	// Continuous Telemetry
	void send_Continuous();
	void send_ContinuousExtended();

	// Onetime Telemetry
	void send_CalibGyro();
	void send_CalibAccel();
	void send_CalibMag();
	// ...
};


extern Telemety telemetry;

#endif
