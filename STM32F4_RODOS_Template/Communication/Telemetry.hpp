#ifndef FLOATSAT_COMMUNICATION_TELEMETRY_HPP_
#define FLOATSAT_COMMUNICATION_TELEMETRY_HPP_

#include "rodos.h"

#include "TelemetryList.hpp"
#include "../hardware/imu.hpp"
#include "../control/AttitudeEstimation.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"


class Telemetry
{
public:

	// @brief Sends telemetry
	void send_Continuous();

	// @brief Sends extended telemetry (only the extended)
	void send_ContinuousExtended();

	// @brief Sends IMU calib telemetry
	void send_CalibIMU();

	// @brief Sends control params telemetry
	void send_ControlParams();
};

// @brief Global telemetry object
extern Telemetry telemetry;

// @brief Global countinuos telemetry topic
extern Topic<TelemetryContinuous> telemetryContinuousTopic;

// @brief Global extended countinuos telemetry topic
extern Topic<TelemetryContinuousExtended> telemetryExtendedContinuousTopic;

// @brief Global IMU calib telemetry topic
extern Topic<TelemetryCalibIMU> telemetryCalibIMUTopic;

// @brief Global control params telemetry topic
extern Topic<TelemetryControlParams> telemetryControlParamsTopic;


#endif
