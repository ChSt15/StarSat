#ifndef FLOATSAT_COMMUNICATION_TELEMETRY_HPP_
#define FLOATSAT_COMMUNICATION_TELEMETRY_HPP_

#include "rodos.h"

#include "TelemetryList.hpp"
#include "../hardware/imu.hpp"
#include "../control/AttitudeEstimation.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"


class Telemetry
{
private:

	bool enable_extendedtelem = false;

	void send_ContinuousExtended();

public:

	// @brief Sends telemetry 
	void send_Continuous();

	// @brief Sends IMU calib telemetry
	void send_CalibIMU(float f);

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

extern Topic<int32_t> Testtopic;
#endif
